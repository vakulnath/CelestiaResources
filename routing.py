import pandas as pd
from datetime import datetime, timedelta
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import plotly.express as px
import plotly.graph_objects as go

def format_time(time_str):
    # Parse the time string
    time_obj = datetime.strptime(time_str, '%H:%M')

    # Format the time based on the hour
    if time_obj.hour < 10:
        return time_obj.strftime('%-H:%M')
    else:
        return time_obj.strftime('%H:%M')

def find_next_available_time(data, requested_time):
    # Adjust the time format for single-digit hours
    time_format = '%H:%M'
    data['Time_dt'] = pd.to_datetime(data['Time'], format='%H:%M').dt.time

    # Convert requested_time to datetime for comparison
    requested_time_dt = datetime.strptime(requested_time, time_format).time()

    if requested_time_dt not in data['Time_dt'].values:
        return "Requested time not found in the dataset", None

    status_at_requested_time = data[data['Time_dt'] == requested_time_dt]['Status'].iloc[0]

    if status_at_requested_time != 'ISS behind':
        return status_at_requested_time, requested_time

    # Iterate through the times in the dataset minute by minute to find the next available time
    current_time = requested_time_dt
    while True:
        # Increment current time by one minute
        current_time = (datetime.combine(datetime.today(), current_time) + timedelta(minutes=1)).time()

        if current_time in data['Time_dt'].values:
            next_status = data[data['Time_dt'] == current_time]['Status'].iloc[0]
            if next_status != 'ISS behind':
                final_time = format_time(current_time.strftime(time_format))
                return status_at_requested_time, final_time

        # Break if we complete a 24-hour cycle
        if current_time == requested_time_dt:
            break

    return "No next available time found within 24 hours", None

def find_next_available(data, requested_time):
    # Adjust the time format for single-digit hours
    time_format = '%H:%M'
    data['Time_dt'] = pd.to_datetime(data['Time'], format='%H:%M').dt.time

    # Convert requested_time to datetime for comparison
    requested_time_dt = datetime.strptime(requested_time, time_format).time()

    if requested_time_dt not in data['Time_dt'].values:
        return "Requested time not found in the dataset", None

    status_at_requested_time = data[data['Time_dt'] == requested_time_dt]['Status'].iloc[0]
    return status_at_requested_time, requested_time

def find_accessible_satellites_modified(data, requested_time):
    # Set display option to avoid scientific notation
    pd.set_option('display.float_format', lambda x: '%.3f' % x)

    time = format_time(requested_time)

    # Filter rows based on adjusted time
    time_rows = data[data['Time'] == time]

    satellite_links = []
    for _, row in time_rows.iterrows():
        for col in data.columns:
            if col.startswith('To') and row[col] and (f'Status{col[2:]}' in data.columns) and row[f'Status{col[2:]}'] == 'in contact':
                from_col = 'From' if col == 'To' else f'From{col[2:]}'
                distance_col = f'Distance{col[2:]}'
                duration_col = f'Duration{col[2:]}'

                # Convert distance to float, which inherently does not use scientific notation
                formatted_distance = float(row[distance_col])

                satellite_links.append({
                    'Time': row['Time'],
                    'From': row[from_col],
                    'To': row[col],
                    'Distance': formatted_distance,  # Use the formatted distance
                    'Status': row[f'Status{col[2:]}'],
                    'Duration': row[duration_col]
                })
    # Remove duplicates if any
    return pd.DataFrame(satellite_links).drop_duplicates()

def calculate_transmission_time(satellite_links, data_size_kbits):
    speed_of_light_kmps = 299792  # Speed of light in kilometers per second

    # Calculate light travel time in seconds and then convert to minutes
    satellite_links['Light Travel Time (minutes)'] = (satellite_links['Distance'] / speed_of_light_kmps) / 60

    # Calculate data transmission time in seconds and then convert to minutes
    satellite_links['Data Transmission Time (minutes)'] = (data_size_kbits / 256) / 60

    # Calculate total transmission time in minutes
    satellite_links['Total Transmission Time (minutes)'] = satellite_links['Light Travel Time (minutes)'] + satellite_links['Data Transmission Time (minutes)']

    return satellite_links

def add_minutes_to_time(time_str, minutes):
    time_format = '%H:%M'
    new_time = datetime.strptime(time_str, time_format) + timedelta(minutes=minutes)
    hour = new_time.hour
    if hour < 10:
        time_format = '%-H:%M'  # Remove leading zero for single digit hours
    else:
        time_format = '%H:%M'   # Retain leading zero for double digit hours
    return new_time.strftime(time_format)


def create_light_travel_time_matrix(satellite_links):
    # Extract unique satellite names
    # Trim spaces from 'From' and 'To' columns
    satellite_links['From'] = satellite_links['From'].str.strip()
    satellite_links['To'] = satellite_links['To'].str.strip()

    # Now create the set of unique satellite names
    satellites = list(set(satellite_links['From']).union(set(satellite_links['To'])))

    # Initialize the matrix with 'float('inf')' (representing unavailable paths)
    matrix = pd.DataFrame((99999), index=satellites, columns=satellites)

    # Populate the matrix with light travel times
    for _, row in satellite_links.iterrows():
        from_sat = row['From']
        to_sat = row['To']
        light_travel_time = int(10*(row['Light Travel Time (minutes)']))
        if from_sat != to_sat:  # Avoid self-loop paths
            matrix.loc[from_sat, to_sat] = light_travel_time
            matrix.loc[to_sat, from_sat] = light_travel_time  # Assuming symmetric travel time

    return matrix

def create_data_model(light_travel_time_matrix, depot_name):
    data = {}
    # Light travel time matrix
    data['time_matrix'] = light_travel_time_matrix.values.tolist()

    # Dynamically create the list of satellite names from the matrix index
    data['satellite_names'] = light_travel_time_matrix.index.tolist()

    data['num_vehicles'] = 1
    # Find the index of ISS in the satellite names and set it as the depot
    data['start'] = data['satellite_names'].index(depot_name)
    #print(data['start'])
    data['end'] = data['satellite_names'].index(depot_name)
    #print(data['end'])
    return data

def create_initial_plot(csv_file, valid_time):
    coordinates_df = pd.read_csv(csv_file)
    coordinates_df['Formatted_Time'] = coordinates_df['Time'].apply(format_time)
    data_for_plot = coordinates_df[coordinates_df['Formatted_Time'] == valid_time]

    if data_for_plot.empty:
        raise ValueError("No data found for the specified valid time")

    plot_data = {
        "Satellite": [],
        "X": [],
        "Y": [],
        "Z": []
    }

    satellite_columns = [col for col in coordinates_df.columns if 'Sat_Name' in col]

    for sat_col in satellite_columns:
        index_postfix = sat_col.split('.')[-1] if '.' in sat_col else ''
        x_col = 'X' + ('.' + index_postfix if index_postfix.isdigit() else '')
        y_col = 'Y' + ('.' + index_postfix if index_postfix.isdigit() else '')
        z_col = 'Z' + ('.' + index_postfix if index_postfix.isdigit() else '')

        if not data_for_plot[sat_col].empty:
            satellite_name = data_for_plot[sat_col].values[0].strip()
            x_value = data_for_plot[x_col].values[0]
            y_value = data_for_plot[y_col].values[0]
            z_value = data_for_plot[z_col].values[0] if z_col in data_for_plot.columns else 0

            # Adjust coordinates for specific satellites
            if satellite_name == "Mro":
                x_value += 1
                y_value += 0.03
                z_value += 0.00  # Adjust Z coordinate as well
            elif satellite_name == "Maven":
                x_value += 2
                y_value += 0.06
                z_value += 0.00  # Adjust Z coordinate
            elif satellite_name == "Odyssey":
                x_value -= 3
                y_value -= 0.03
                z_value -= 0.00  # Adjust Z coordinate

            plot_data["Satellite"].append(satellite_name)
            plot_data["X"].append(x_value)
            plot_data["Y"].append(y_value)
            plot_data["Z"].append(z_value)

    fig = go.Figure(data=[go.Scatter3d(
        x=plot_data["X"],
        y=plot_data["Y"],
        z=plot_data["Z"],
        mode='markers+text',
        text=plot_data["Satellite"],
        marker=dict(size=5)
    )])

    fig.update_layout(
        title="3D positions of Satellites at " + valid_time,
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        )
    )

    satellite_coords = {
        name: (x, y, z) for name, x, y, z in zip(
            plot_data["Satellite"], plot_data["X"], plot_data["Y"], plot_data["Z"]
        )
    }

    return fig, satellite_coords
    
def plot_solution_on_graph(solution, manager, routing, data, graph, satellite_coords, color_mapping, offset_factor=1):
    vehicle_id = 0  # If you have multiple routes, this needs to be iterated over each vehicle/route

    index = routing.Start(vehicle_id)
    path = []
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        satellite_name = data['satellite_names'][node]
        path.append(satellite_coords[satellite_name])
        index = solution.Value(routing.NextVar(index))

    # Apply an offset to each coordinate to reduce overlap
    path_with_offset = [(x + offset_factor * vehicle_id, y + offset_factor * vehicle_id, z + offset_factor * vehicle_id) for x, y, z in path]

    satellite_name = data['satellite_names'][routing.Start(vehicle_id)]
    route_color = color_mapping.get(satellite_name, 'grey')  # Get the color for the route

    x_coords, y_coords, z_coords = zip(*path_with_offset)

    # Plot the route with the specific color
    graph.add_trace(go.Scatter3d(
        x=x_coords,
        y=y_coords,
        z=z_coords,
        mode='lines',
        line=dict(width=2, color=route_color),
        name=satellite_name
    ))

    # Optionally add markers or arrows here with the specific color if needed

    return graph

def get_solution_path(solution, manager, routing, data):
    path = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        path.append(data['satellite_names'][node])
        index = solution.Value(routing.NextVar(index))

    last_node = manager.IndexToNode(index)
    path.append(data['satellite_names'][last_node])
    return path

def main():
    # Load ISS and Mars data
    iss_data = pd.read_csv('/Users/vakulnath/Desktop/files/iss.csv')
    mars_data = pd.read_csv('/Users/vakulnath/Desktop/files/mars.csv')
    coordinates = '/Users/vakulnath/Desktop/files/coordinates.csv' 
    routes = '/Users/vakulnath/Desktop/files/routes3.txt'

    requested_time = '0:38'

    while requested_time != "0:39":
        solutions = []
        # Check the status of the ISS at the requested time
        status, valid_time = find_next_available(iss_data, requested_time)

        data_size_kbits = 1000  # example data packet size

        # Handling for ISS
        if status == 'ISS behind':
            #solutions.append("ISS is behind Earth.")
            iss_transmission_times = pd.DataFrame()
        else:
            iss_satellite_links = find_accessible_satellites_modified(iss_data, requested_time)
            if not iss_satellite_links.empty:
                iss_transmission_times = calculate_transmission_time(iss_satellite_links, data_size_kbits)
            else:
                iss_transmission_times = pd.DataFrame()

        # Handling for Mars
        mars_satellite_links = find_accessible_satellites_modified(mars_data, requested_time)
        if not mars_satellite_links.empty:
            mars_transmission_times = calculate_transmission_time(mars_satellite_links, data_size_kbits)
        else:
            mars_transmission_times = pd.DataFrame()
            #solutions.append("No accessible Mars satellites at this time.")

        # Check if there is any satellite link data available before proceeding
        if not iss_transmission_times.empty or not mars_transmission_times.empty:
            links = pd.concat([iss_transmission_times, mars_transmission_times])
            light_travel_time_matrix = create_light_travel_time_matrix(links)
            print(light_travel_time_matrix)

            color_mapping = {
                'ISS': 'black',
                'Express': 'red',
                'Mro': 'green',
                'Maven': 'orange',
                'Odyssey': 'blue'
            }

            fig, satellite_coords = create_initial_plot(coordinates, valid_time)

            for depot_name in light_travel_time_matrix.index:
                updated_data = create_data_model(light_travel_time_matrix, depot_name)
                
                manager = pywrapcp.RoutingIndexManager(
                            len(updated_data['time_matrix']),
                            updated_data['num_vehicles'],
                            [updated_data['start']],
                            [updated_data['end']]
                        )

                routing = pywrapcp.RoutingModel(manager)
                transit_callback_index = routing.RegisterTransitCallback(lambda from_index, to_index: updated_data['time_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])
                routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

                search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC

                solution = routing.SolveWithParameters(search_parameters)

                if solution:
                    current_solution = get_solution_path(solution, manager, routing, updated_data)
                    solutions.append(current_solution)
                    graph = plot_solution_on_graph(solution, manager, routing, updated_data, fig, satellite_coords, color_mapping)
                else:
                    solutions.append(f"{depot_name}: No solution found")
        else:
            solutions.append("No satellite links available for routing.")

        print(solutions)
        graph.show()
        #with open(routes, 'a') as f:
        #    f.write(f'{requested_time},{valid_time}, ; {solutions}\n')

        # Update requested_time
        requested_time = add_minutes_to_time(requested_time, 1)
        print(f"Requested time updated to {requested_time}")

def print_solution(manager, routing, solution, data):
    routes = []
    plan_output = 'Route for satellite transmission: '
    index = routing.Start(0)
    while True:
        plan_output += '{} -> '.format(data['satellite_names'][manager.IndexToNode(index)])
        if routing.IsEnd(index):
            break
        index = solution.Value(routing.NextVar(index))

    # Remove the last ' -> ' from the string
    plan_output = plan_output.rstrip(' -> ')
    return plan_output

if __name__ == '__main__':
    main()
