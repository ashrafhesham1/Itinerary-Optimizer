from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

class optimizer:
    def __init__(self):
        pass

    def genrate_itinerary(self,coordinates, start = -1, num_days = 1):
        '''
        given coordinates of some places it generates an Itinerary to visit them
        Args:
            coordinates: (list) of tuples each indeicate the coordinates of the destinations to visit
            start: (int) the index of the place that is the start of the itinerary
            num_days: (int) number of days that the itinerary covers
        Returns:
            itinerary: (list) of lists each represent the location to be visited in a single day
        '''
        self._generate_data_object(coordinates, num_days ,start)
        min_stops = 2
        min_stops = min(min_stops if start >= 0 else min_stops + 1, (len(coordinates) // num_days)+1)
        max_stops = len(coordinates)

        return self._vrp(start, min_stops=min_stops, max_stops=max_stops)
 
    def _vrp(self,start, max_vehicle_capacity = 9223372036854775807,min_stops = 0, max_stops = 1000):
        '''
        solves the vehicle routing problem
        Args:
            start: (int) the index of the place that is the start of the itinerary
            max_vehicle_capacity: (int) the maximum capacity of all vehicles combined
            min_stops: (int) the minimum numbers of cities each vehicle should visit
            max_stops: (int) the maximum numbers of cities each vehicle should visit
        Returns:
            itinerary: (list) of lists each represent the location to be visited in a single day
        '''
        # create routing model
        self.manager = pywrapcp.RoutingIndexManager(len(self.data_obj['distance_matrix']),self.data_obj['num_vehicles'], self.data_obj['depot'])        
        routing = pywrapcp.RoutingModel(self.manager)

        # Add Counter Constrain
        # set up the call back function
        plus_one_callback_index = routing.RegisterUnaryTransitCallback(lambda index : 1)

        dimension_name = 'Counter'
        routing.AddDimension(
        plus_one_callback_index,
        0,  # null capacity slack
        max_stops,  # vehicle maximum capacities
        True,  # start cumul to zero
        dimension_name)
        counter_dimension = routing.GetDimensionOrDie(dimension_name)
        for vehicle_id in range(self.data_obj['num_vehicles']):
            index = routing.End(vehicle_id)
            counter_dimension.CumulVar(index).SetRange(min_stops, max_stops)

        # Add Distance constraint.
        #setup the get distance function to be the distance call back
        transit_callback_index = routing.RegisterTransitCallback(self._get_distance)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            max_vehicle_capacity,  
            True,  # start cumul to zero
            dimension_name)


        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(10000)
        
        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # save the solution to a list
        itinerary = []
        for route_nbr in range(routing.vehicles()):
            index = routing.Start(route_nbr)
            route = [self.manager.IndexToNode(index)] if start >= 0 else []
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index))
                node = self.manager.IndexToNode(index)
                route.append(node if start >= 0 else node-1)
            itinerary.append(route[:-1])
        
        return itinerary
    
    def _calculate_distance(self, p1, p2):
        '''
        calculate Haversine distance between two points
        Args:
            p1: (tuple) first point
            p2: (tuple) second point
        Returns:
            distance: (float) the Haversine distance between the two points
        '''
            
        lon1, lat1 = p1
        lon2, lat2 = p2
        
        # Earth's radius in kilometers
        R = 6371 

        # convert latitude and longitude to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # calculate Haversine formula
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        return distance
  
    def _generate_data_object(self, coordinates, num_vehicles, depot):
        '''
        calculate distance between each two places construction distance matrix and compine it with the 
        rest of the required data into a data object
        Args:
            coordinates: (list) of tuples each indeicate the coordinates of the destinations to visit
            num_vehicles: (int) The number of vehicles in the fleet (the number of the days the trip will take).
            depot:(int) The index of the depot, the location where all vehicles start and end their routes.
        Returns:
        '''
        # construct distance matrix
        num_locations = len(coordinates)
        distance_matrix = []
        if depot >= 0:
            for i in range(num_locations):
                row = []
                for j in range(num_locations):
                    distance = int(self._calculate_distance(coordinates[i], coordinates[j]) * 10)
                    row.append(distance)
                distance_matrix.append(row)
        else: #choose arbitary start
            distance_matrix.append([0]*(num_locations + 1))
            for i in range(num_locations):
                row = [0]
                for j in range(num_locations):
                    distance = int(self._calculate_distance(coordinates[i], coordinates[j]) * 10)
                    row.append(distance)
                distance_matrix.append(row)

        # build data object
        data_obj = {}
        data_obj['distance_matrix'] = distance_matrix
        data_obj['num_vehicles'] = num_vehicles
        data_obj['depot'] = max(0, depot)

        self.data_obj = data_obj
    
    def _get_distance(self, from_idx, to_idx):
        '''
        get the distance between two ponints given their inner indices in the manager object using the distance matrix
        Args:
            from_idx: (int) inner index of the first node
            to_idx: (int) inner index of the first node
        Returns:
            distance: (float) the ecludian distance between the two points
        '''
        from_node = self.manager.IndexToNode(from_idx)
        to_node = self.manager.IndexToNode(to_idx)

        return self.data_obj['distance_matrix'][from_node][to_node]

    def generate_location_box(p, d):
        """
        Calculates the bounding box of a location given its latitude, longitude,
        and a distance in kilometers.

        Args:
            p: (tuple) A tuple containing the longitude and latitude of the location
                    (in that order) in degrees.
            d: (float) The distance from the location in kilometers.

        Returns:
            loc_box: (dict) A dictionary containing the bounding box coordinates of the location
                in degrees. The dictionary has four keys: 'min_lon', 'max_lon', 'min_lat',
                and 'max_lat'representing the minimum and maximum
                longitude and latitude, respectively, that define the bounding box.
        """
        EARTH_RADIUS = 6371.01

        # Convert coordinates from degrees to radians
        lon, lat = map(math.radians, p)

        # Calculate the new latitudes and longitudes
        lat1 = math.asin(math.sin(lat) * math.cos(d / EARTH_RADIUS) +
                        math.cos(lat) * math.sin(d / EARTH_RADIUS) * math.cos(0))
        lon1 = lon + math.atan2(math.sin(0) * math.sin(d / EARTH_RADIUS) * math.cos(lat),
                                math.cos(d / EARTH_RADIUS) - math.sin(lat) * math.sin(lat1))
        
        lat2 = math.asin(math.sin(lat) * math.cos(d / EARTH_RADIUS) +
                        math.cos(lat) * math.sin(d / EARTH_RADIUS) * math.cos(math.pi / 2))
        lon2 = lon + math.atan2(math.sin(math.pi / 2) * math.sin(d / EARTH_RADIUS) * math.cos(lat),
                                math.cos(d / EARTH_RADIUS) - math.sin(lat) * math.sin(lat2))

        lat3 = math.asin(math.sin(lat) * math.cos(d / EARTH_RADIUS) +
                        math.cos(lat) * math.sin(d / EARTH_RADIUS) * math.cos(math.pi))
        lon3 = lon + math.atan2(math.sin(math.pi) * math.sin(d / EARTH_RADIUS) * math.cos(lat),
                                math.cos(d / EARTH_RADIUS) - math.sin(lat) * math.sin(lat3))

        lat4 = math.asin(math.sin(lat) * math.cos(d / EARTH_RADIUS) +
                        math.cos(lat) * math.sin(d / EARTH_RADIUS) * math.cos(3 * math.pi / 2))
        lon4 = lon + math.atan2(math.sin(3 * math.pi / 2) * math.sin(d / EARTH_RADIUS) * math.cos(lat),
                                math.cos(d / EARTH_RADIUS) - math.sin(lat) * math.sin(lat4))
        
        # getting minimum and maximum coordinates that shapes the box
        loc_box = {}
        loc_box['min_lon'] = min(math.degrees(lon1), math.degrees(lon2), math.degrees(lon3), math.degrees(lon4))
        loc_box['max_lon'] = max(math.degrees(lon1), math.degrees(lon2), math.degrees(lon3), math.degrees(lon4))
        loc_box['min_lat'] = min(math.degrees(lat1), math.degrees(lat2), math.degrees(lat3), math.degrees(lat4))
        loc_box['max_lat'] = max(math.degrees(lat1), math.degrees(lat2), math.degrees(lat3), math.degrees(lat4))


        return loc_box

