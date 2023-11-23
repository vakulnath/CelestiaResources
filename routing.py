from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    data = {}
    # Distance matrix
    data['distance_matrix'] = [
        # ISS,       Express,     MRO,         Odyssey,      Maven
        [0, 82012151, 376807247, 376806829, 376812199],    # ISS
        [82012151, 0, 446440712, 46440099, 446445609],    # Express
        [376807247, 446440712, 0, float('inf'), 4989],    # MRO
        [376806829, 46440099, float('inf'), 0, 8184],     # Odyssey
        [376812199, 446445609, 4989, 8184, 0]             # Maven
    ]

    # Data rates in Mbps
    data['data_rates'] = {
        'ISS': 600,
        'Express': {'min': 0.057, 'max': 0.228},
        'MRO': 2,
        'Odyssey': {'min': 0.128, 'max': 0.256},
        'MAVEN': 2.048
    }

    # Satellite names
    data['satellite_names'] = ['ISS', 'Express', 'MRO', 'Odyssey', 'MAVEN']

    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def get_segment_data_rate(from_satellite, to_satellite, data_rates):
    # Assuming the minimum data rate for satellites with a range
    from_rate = data_rates[from_satellite]['min'] if isinstance(data_rates[from_satellite], dict) else data_rates[from_satellite]
    to_rate = data_rates[to_satellite]['min'] if isinstance(data_rates[to_satellite], dict) else data_rates[to_satellite]
    return min(from_rate, to_rate)

def composite_cost_callback(from_index, to_index, data, manager):
    from_satellite = data['satellite_names'][manager.IndexToNode(from_index)]
    to_satellite = data['satellite_names'][manager.IndexToNode(to_index)]
    distance = data['distance_matrix'][from_index][to_index]
    rate = get_segment_data_rate(from_satellite, to_satellite, data['data_rates'])
    return 1 / (distance * rate) if distance != float('inf') else float('inf')

def main():
    data_volume_megabits = 10
    data = create_data_model()

    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    composite_cost_index = routing.RegisterTransitCallback(
        lambda from_index, to_index: composite_cost_callback(from_index, to_index, data, manager)
    )

    routing.SetArcCostEvaluatorOfAllVehicles(composite_cost_index)

    # Add constraints to prevent direct communication between MRO and Odyssey
    mro_index = manager.NodeToIndex(data['satellite_names'].index('MRO'))
    odyssey_index = manager.NodeToIndex(data['satellite_names'].index('Odyssey'))
    
    for node in range(routing.Size()):
        if node != mro_index and node != odyssey_index:
            routing.solver().Add(routing.NextVar(mro_index) != node)
            routing.solver().Add(routing.NextVar(odyssey_index) != node)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    print("Starting the solver...")
    solution = routing.SolveWithParameters(search_parameters)
    print("Solver finished.")

    if solution:
        print_solution(manager, routing, solution, data['satellite_names'], data['data_rates'], data, data_volume_megabits)
    else:
        print("No solution found.")

SPEED_OF_LIGHT_KM_PER_S = 299792  # Speed of light in kilometers per second

def print_segment_info(from_satellite, to_satellite, data_rate, distance, total_data_volume_megabits):
    # Calculate the time to transmit the fixed data volume at the given data rate in seconds
    transmission_time_seconds = total_data_volume_megabits / data_rate * 8 * 60  # Convert to seconds
    # Calculate the physical transmission time based on distance
    physical_transmission_time_seconds = distance / SPEED_OF_LIGHT_KM_PER_S 
    total_time_seconds = transmission_time_seconds + physical_transmission_time_seconds

    # Convert total time from seconds to hours
    total_time_hours = total_time_seconds / 3600

    print(f"Segment {from_satellite} to {to_satellite}: "
          f"Rate = {data_rate} Mbps, "
          f"Transmission Time = {transmission_time_seconds / 60:.2f} minutes, "
          f"Physical Transmission Time = {physical_transmission_time_seconds:.2f} seconds, "
          f"Total Time = {total_time_hours:.2f} hours for {total_data_volume_megabits} Megabits")

def print_solution(manager, routing, solution, satellite_names, data_rates, data, total_data_volume_megabits):
    print('Objective: longest path with maximum data throughput')
    index = routing.Start(0)
    plan_output = 'Route for satellite network:\n'
    min_data_rate = float('inf')

    while not routing.IsEnd(index):
        from_index = index
        index = solution.Value(routing.NextVar(index))
        to_index = index

        if not routing.IsEnd(index):
            from_satellite = satellite_names[manager.IndexToNode(from_index)]
            to_satellite = satellite_names[manager.IndexToNode(to_index)]
            segment_rate = get_segment_data_rate(from_satellite, to_satellite, data_rates)
            segment_distance = data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
            min_data_rate = min(min_data_rate, segment_rate)
            print_segment_info(from_satellite, to_satellite, segment_rate, segment_distance, total_data_volume_megabits)

            plan_output += f' {from_satellite} ->'

    satellite_name = satellite_names[manager.IndexToNode(index)]
    plan_output += f' {satellite_name}\n'
    
    print(plan_output)

    total_data_capacity = "Minimum Data Rate of the Network: {} Mbps".format(min_data_rate)
    print(total_data_capacity)
    duration_seconds = total_data_volume_megabits / min_data_rate # Convert to minutes
    print(f"Total Time to transmit {total_data_volume_megabits} Megabits: {duration_seconds:.2f} hours")
    
if __name__ == '__main__':
    main()


