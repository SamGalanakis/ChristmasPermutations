import itertools
from tqdm import tqdm
import pickle
from ortools.constraint_solver import pywrapcp
import ortools
from ortools.constraint_solver import routing_enums_pb2








# def permutation_distance(a,b):
#     b_length = len(b)
#     for i in range(1,1+len(b))[::-1]:
#         if a[-i:] == b[:i]:
#             return b_length-i
#     return b_length
    

# for entries in range(4,8):
#     values = [str(i) for i in range(0,entries)]

#     replace_dict = {
#         "0": '🎅', 
#         "1": '🤶', 
#         "2": '🦌', 
#         "3": '🧝', 
#         "4": '🎄', 
#         "5": '🎁', 
#         "6": '🎀', 
#     }

#     permutations = list(itertools.permutations(values,len(values)))

#     def get_distance_matrix(permutations,measure):
#         distance_matrix = []
#         for permutation in tqdm(permutations):
#             dist_list = []
#             for inner_permutation in permutations:
#                 dist = measure(permutation,inner_permutation)
#                 dist_list.append(dist)
#             distance_matrix.append(dist_list)
#         return distance_matrix
#     distance_matrix = get_distance_matrix(permutations,permutation_distance)
#     with open(f"distance_matrix_{entries}.pcl","wb") as f:
#         pickle.dump(distance_matrix,f)





def create_data_model():
    """Stores the data for the problem."""
    data = {}

    with open("distance_matrix_6.pcl","rb") as f:
        distance_matrix = pickle.load(f)
    n_permutations = len(distance_matrix)


    # Extra dummy row for stard/end point that is big distance from everything (so not used as shortcut)
    distance_matrix.insert(0,[0]*n_permutations)
    
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 3
    data['depot'] = 0
   
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))



def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        300000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.seconds = 10

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')


if __name__ == '__main__':
    main()