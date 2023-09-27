function[] = Drone_Delivery_Simulation()

    customers = 30;
    drones = 3;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Look at waiting time function

    % Generates customer locations
    [customer_locations] = generate_customer_locations(customers);

    % Calculates distances
    [customer_distances, customer_distances_depot] = calculate_distances(customer_locations, customers);

    % Calculates the max flight time
    [max_flight_time] = calculate_flight_time(customer_distances_depot, customers);

    % Generates initial solution
    [soln, b_customers_in_route] = generate_initial_solution(customers, drones, customer_distances, customer_distances_depot);

    % ECA Algorithm - Elliptical customer assignment
    [soln] = eca_algorithm(soln, customers, drones, customer_locations, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);

    % TDRA algorithm
    [soln] = tdra_algorithm(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations);

   
    % Debug output
    soln
    calculate_waiting_time(soln, customer_distances, customer_distances_depot)

    % Plot the solution
    plot_stuff(soln, customer_locations);

end

%-----------------------------------------------------------------------%
%-----------------------------------------------------------------------%
%-----------------------------------------------------------------------%

function[customer_locations] = generate_customer_locations(customers)

    % x and y limits
    x_limits = [-50, 50];
    y_limits = [-50, 50];
    
    % Initilizes the customer location matrix
    customer_locations = zeros(customers, 2);

    % Generates the locations
    rng(35467, 'twister')
    for     icustomer = 1 : customers
            x_location = rand() * (x_limits(2) - x_limits(1)) + x_limits(1);
            y_location = rand() * (y_limits(2) - y_limits(1)) + y_limits(1);
            
            customer_locations(icustomer, 1) = x_location;
            customer_locations(icustomer, 2) = y_location;
    end

end

%-----------------------------------------------------------------------%

function[customer_distances, customer_distances_depot] = calculate_distances(customer_locations, customers)

    customer_distances = zeros(customers, customers);
    customer_distances_depot = zeros(customers + 2, customers + 2);
    customer_locations_depot = [0, 0; customer_locations; 0, 0];

    % Makes the matrix for customer distances where each row is the departing customer and the column is the arriving customer.
    for     ideparture = 1 : customers
            for     iarrival = 1 : customers
                    customer_distances(ideparture, iarrival) = sqrt((customer_locations(iarrival, 1) - customer_locations(ideparture, 1)) ^ 2 + (customer_locations(iarrival, 2) - customer_locations(ideparture, 2))^2);
            end
    end

    % Customer distances with depot included
    for     ideparture = 1 : customers + 2
            for     iarrival = 1 : customers + 2
                    customer_distances_depot(ideparture, iarrival) = sqrt((customer_locations_depot(iarrival, 1) - customer_locations_depot(ideparture, 1)) ^ 2 + (customer_locations_depot(iarrival, 2) - customer_locations_depot(ideparture, 2))^2);
            end
    end
end

%-----------------------------------------------------------------------%

function[max_flight_time] = calculate_flight_time(customer_distances_depot, customers)

    % Algorithm 9 which finds the max flight time for a drone trip

    % Sets up some parameters for the algorithm
    coverage_percentage = .75;
    epsilon = 1;
    flight_percentage = 0;
    max_flight_time = 0;

    % Continues running until the percentage of successful flights is above or equal to our set percentage
    while flight_percentage < coverage_percentage

        % Updates the max flight time and sets the successful flights back to 0
        max_flight_time = max_flight_time + epsilon;
        num_successful_flights = 0;
    
        % Loops over all posible delivery points
        for j = 1 : customers

            % Loops over all possible leaving points
            for i = [0 : j - 1, j + 1 : customers]

                % Loops over all possible returning points
                for s = [1 : j - 1, j + 1 : customers, 0]

                    % Finds the flight time for the current route
                    flight_time = (customer_distances_depot(i + 1, j + 1) + customer_distances_depot(j + 1, s + 1)) / 1.5;

                    % If the flight time is less than the current max flight time, we count it as a successful flight
                    if  flight_time <= max_flight_time
                        num_successful_flights = num_successful_flights + 1;
                    end
                end
            end
        end

        % Calculates the percentage of successful flights
        flight_percentage = num_successful_flights / ((customers ^ 2) * (customers + 1));

    end



end

%-----------------------------------------------------------------------%

function[best_soln, b_customers_in_route] = generate_initial_solution(customers, drones, customer_distances, customer_distances_depot)
    
    % Initilizes the solution parts and generates intial solution for part 1
    soln.part1 = [0, randperm(customers), 0];
    soln.part2 = -1 * ones(drones - 1);
    soln.part3 = -1 * ones(drones - 1);
    soln.part4 = -1 * ones(drones - 1);

    % Number of times to do 2 - opt
    i_max = 10;

    % Initilizes temp stuff
    threshhold_temp = 100;
    temp_multiplier = .99;
    current_temp = 250;

    % Sets up the current solution
    curr_soln = soln;

    % Sets up the best solution
    best_soln = soln;


    % Calculates waiting time for initial solution and saves it as the current best
    [best_waiting_time] = calculate_waiting_time(soln, customer_distances, customer_distances_depot);

    % Simulated Annealing
     while  current_temp > threshhold_temp
            for     i = 1 : i_max

                    % "Well known" 2 - opt
                    first_index = randi([2, length(soln.part1) - 1]);
                    second_index = randi([2, length(soln.part1) - 1]);
                    while   first_index == second_index
                            second_index = randi([2, length(soln.part1) - 1]);
                    end
                    first_stop = soln.part1(first_index);
                    second_stop = soln.part1(second_index);
                    curr_soln = soln;
                    curr_soln.part1(first_index) = second_stop;
                    curr_soln.part1(second_index) = first_stop;

                    % Replaces current best if new solution is better
                    [curr_waiting_time] = calculate_waiting_time(curr_soln, customer_distances, customer_distances_depot);
                    if      curr_waiting_time < best_waiting_time
                            best_waiting_time = curr_waiting_time;
                            best_soln = curr_soln;
                    end

                    % Boltzmann probability thing
                    [soln_waiting_time] = calculate_waiting_time(soln, customer_distances, customer_distances_depot);
                    if      exp(-(abs(soln_waiting_time - curr_waiting_time)) / current_temp) > rand(0, 1)
                            soln = curr_soln;
                    end
            end
            current_temp = temp_multiplier * current_temp;
     end

    % Makes the boolean vector that is used in the feasibility function
    b_customers_in_route = ones(1, customers);

    %fprintf('\nFinal waiting time: %0.2f\n\n', best_waiting_time);
end

%-----------------------------------------------------------------------%

function[best_soln] = tdra_algorithm(curr_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations)

    % The tdra algorithm picker
    
    % Sets the current solution to be the best
    best_soln = curr_soln;
    curr_soln

    % Sets the number of heruisics that we have
    num_heuristics = 10;

    % Sets up the weights
    weight_info = weight_init(num_heuristics);

    % Sets the max iterations for the amount of algorithms to pick
    max_iterations = 500;
    non_improving_iterations = 0;

    % Initilizes the temperature stuff for the boltzman probability
    temp_multiplier = .999;
    current_temp = 5000;

    % Set up stuff for the plots
    best_waiting_times = zeros(1, max_iterations);
    current_waiting_times = zeros(1, max_iterations);
    prime_waiting_times = zeros(1, max_iterations);
    heuristic_waiting_times = zeros(num_heuristics, max_iterations);
    
    % Main loops of the TDRA algorithm
    for i = 1 : max_iterations

        % Output for the current iteration
        fprintf("Iteration: %d\n", i);

        % selects the heuristic to use
        heuristic_to_use = select_heuristic(weight_info);

        % Applies the heuristic to get soln prime
        soln_prime = apply_heuristic(curr_soln, heuristic_to_use, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations);

        soln_prime

        % Updates the scores and every 50 iterations the weights
        weight_info = update_weights(weight_info, i, heuristic_to_use, curr_soln, soln_prime, best_soln, customer_distances, customer_distances_depot);

        % Checks if it is the new best solution
        if  calculate_waiting_time(soln_prime, customer_distances, customer_distances_depot) < calculate_waiting_time(best_soln, customer_distances, customer_distances_depot)
            best_soln = soln_prime;
            non_improving_iterations = 0;
            %fprintf('Solution was better on iteration %d from heuristic %d\n', i, heuristic_to_use)
        else
            non_improving_iterations = non_improving_iterations + 1;
        end

        
        % Checks if we should accept it as the new current solution based on boltzman
        if  exp(-(abs(calculate_waiting_time(curr_soln, customer_distances, customer_distances_depot) - calculate_waiting_time(soln_prime, customer_distances, customer_distances_depot))) / current_temp) > rand(0, 1)
            curr_soln = soln_prime;
            %fprintf('Solution was chosen as new current on iteration %d from heuristic %d\n', i, heuristic_to_use)
        end
        

        % Updates the temperature
        current_temp = current_temp * temp_multiplier;

        % Updates the plot stuff
        best_waiting_times(i) = calculate_waiting_time(best_soln, customer_distances, customer_distances_depot);
        current_waiting_times(i) = calculate_waiting_time(curr_soln, customer_distances, customer_distances_depot);
        prime_waiting_time = calculate_waiting_time(soln_prime, customer_distances, customer_distances_depot);
        prime_waiting_times(i) = prime_waiting_time;
        heuristic_waiting_times(heuristic_to_use, i) = prime_waiting_time;

        % Shake heuristic
        if  non_improving_iterations == (customers * 20)                  % 20 is a randomly chosen coefficient

            % Output for if the shake is being run
            fprintf("Shake\n");

            for n = 1 : 50                                                % 50 is a randomly chosen coefficient
                if  randi([1,2]) == 1
                    curr_soln = heuristic_general_assignment(curr_soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
                else
                    curr_soln = heuristic_wild_change(curr_soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
                end
                if  calculate_waiting_time(curr_soln, customer_distances, customer_distances_depot) < calculate_waiting_time(best_soln, customer_distances, customer_distances_depot)
                    best_soln = curr_soln;
                end
            end
            non_improving_iterations = 0;
        end

        
    end

    % Makes the colors
    rgb_colors = [0,0,0; 1,0,0; 0,1,0; 0,0,1; 1,1,0; 1,0,1; 0,1,1; .5,0,0; 0,.5,0; 0,0,.5];
    

    % Plots the waiting times
    figure(2);
    clf;
    hold on;
    plot(best_waiting_times, 'b');
    plot(current_waiting_times, 'r');
    plot(prime_waiting_times, 'g');
    legend('Best Solution', 'Current Solution', 'Prime Solution')
    title('Waiting Time vs. Iteration')
    xlabel('Iteration')
    ylabel('Waiting Time')
    hold off

    % Plots the weights
    figure(3);
    clf;
    hold on;
    for i = 1 : num_heuristics
        plot(weight_info.plotting(:, i), 'Color', rgb_colors(i, :));
    end
    legend('Two-opt', 'Three-opt', 'Greedy', 'Origin-Destination', 'General Assignment', 'Drone Planner', 'Wild Change', 'Truck Reorder', 'Truck Outlier Removal', 'Drone-to-Truck');
    title('Weight vs. Segment Counter');
    xlabel('Segment Counter');
    ylabel('Weight');
    hold off;


    % Plots the amount of times that each heuristic was used
    figure(4);
    clf;
    bar(weight_info.times);
    set(gca,'xticklabel',{'Two-opt', 'Three-opt', 'Greedy', 'Origin-Destination', 'General Assignment', 'Drone Planner', 'Wild Change', 'Truck Reorder', 'Truck Outlier Removal', 'Drone-to-Truck'});
    title('Number of Times each Heuristic was Used');

    % Plots the waiting times for each heuristic
    figure(5);
    clf;
    hold on;
    for i = 1 : num_heuristics
        plot(heuristic_waiting_times(i, :), 'o', 'Color', rgb_colors(i, :));
    end
    legend('Two-opt', 'Three-opt', 'Greedy', 'Origin-Destination', 'General Assignment', 'Drone Planner', 'Wild Change', 'Truck Reorder', 'Truck Outlier Removal', 'Drone-to-Truck');
    title('Waiting Time vs. Iteration');
    xlabel('Iteration');
    ylabel('Waiting Time');
    hold off;

end

%-----------------------------------------------------------------------%

function[weight_info] = weight_init(num_heuristics)
    
    % Sets up the weights for the algorithms
    weight_info.scores = zeros(1, num_heuristics);
    weight_info.times = zeros(1, num_heuristics);
    weight_info.weights = (1 / num_heuristics) * ones(1, num_heuristics);
    weight_info.gamma = 0.2;
    weight_info.segment_counter = 1;

    % Saves the weights for plotting
    weight_info.plotting(1, :) = weight_info.weights;
  
end

%-----------------------------------------------------------------------%

function[weight_info] = update_weights(weight_info, i, heuristic_to_use, curr_soln, soln_prime, best_soln, customer_distances, customer_distances_depot)
    
    % Takes in the current weight info and updates it every 50 iterations of the TDRA
    
    % updates the scores based on wether the new solution did better than the best or current
    if  calculate_waiting_time(soln_prime, customer_distances, customer_distances_depot) < calculate_waiting_time(best_soln, customer_distances, customer_distances_depot)
        weight_info.scores(heuristic_to_use) = weight_info.scores(heuristic_to_use) + 2;
    elseif calculate_waiting_time(soln_prime, customer_distances, customer_distances_depot) < calculate_waiting_time(curr_soln, customer_distances, customer_distances_depot)
        weight_info.scores(heuristic_to_use) = weight_info.scores(heuristic_to_use) + 1;
    end

    % Updates the number times the heuristic was used
    weight_info.times(heuristic_to_use) = weight_info.times(heuristic_to_use) + 1;

    % Updates the weights and segment counter if it is a multiple of 50 iteration
    if  mod(i, 50) == 0
        weight_info.weights = (weight_info.weights * (1 - weight_info.gamma)) + (weight_info.gamma * (weight_info.scores ./ weight_info.times));
        weight_info.segment_counter = weight_info.segment_counter + 1;
    end

    % Saves the weights into a matrix for plotting
    weight_info.plotting(weight_info.segment_counter, :) = weight_info.weights;

end

%-----------------------------------------------------------------------%

function[heuristic_to_use] = select_heuristic(weight_info)

    % Selects the heuristic to use next

    % Finds the probablities of each of the heuristics
    probabilites = weight_info.weights / sum(weight_info.weights);

    % Makes the probabilities into segments that start at 0 and end at 1 with each portion taking up the probabilty
    pie_segments = zeros(1, length(probabilites));
    for i = 1 : length(probabilites) 
        if  i == 1
            pie_segments(i) = probabilites(i);
        else
            pie_segments(i) = pie_segments(i - 1) + probabilites(i);
        end
    end

    % Generates the random number
    rand_num = rand();

    % Picks the heuristic based on the random number
    for pie_slice = 1 : length(pie_segments)
        if  rand_num < pie_segments(pie_slice)
            heuristic_to_use = pie_slice;
            break
        end
    end

end

%-----------------------------------------------------------------------%

function[soln_prime] = apply_heuristic(soln, heuristic_to_use, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations)

    % Applies the selected heuristic
    
    % Uses the heuristics corresponding to the number
    if  heuristic_to_use == 1
        fprintf('Chose heuristic: Two-opt\n');
        soln_prime = heuristic_two_opt(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 2
        fprintf('Chose heuristic: Three-opt\n');
        soln_prime = heuristic_three_opt(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 3
        fprintf('Chose heuristic: Greedy\n');
        soln_prime = heuristic_greedy(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 4
        fprintf('Chose heuristic: Origin-Destination\n');
        soln_prime = heuristic_origin_destination(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 5
        fprintf('Chose heuristic: General Assignment\n');
        soln_prime = heuristic_general_assignment(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 6
        fprintf('Chose heuristic: Drone Planner\n');
        soln_prime = heuristic_drone_planner(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 7
        fprintf('Chose heuristic: Wild Change\n');
        soln_prime = heuristic_wild_change(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 8
        fprintf('Chose heuristic: Truck Reorder\n');
        soln_prime = heuristic_truck_reorder(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations);
    elseif heuristic_to_use == 9
        fprintf('Chose heuristic: Truck Outlier Removal\n');
        soln_prime = heuristic_truck_outlier_removal(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    elseif heuristic_to_use == 10
        fprintf('Chose heuristic: Drone to Truck\n');
        soln_prime = heuristic_drone_to_truck(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations);

        
        
    end   

end

%-----------------------------------------------------------------------%

function[best_soln] = eca_algorithm(soln, customers, drones, customer_locations, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % Generates the ellipse

    % Initilizes the D matrix and makes the C matrix
    D_matrix = ones(customers + 1, 6);
    for     i = 1 : 5
            D_matrix(1, i) = 0;
    end
    C_matrix = [0,0,2,0,0,0;0,-1,0,0,0,0;2,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];

    % Makes the D matrix
    for     icustomer = 1 : customers
            D_matrix(icustomer + 1, 1) = (customer_locations(icustomer, 1))^2;
            D_matrix(icustomer + 1, 2) = customer_locations(icustomer, 1) * customer_locations(icustomer, 2);
            D_matrix(icustomer + 1, 3) = (customer_locations(icustomer, 2))^2;
            D_matrix(icustomer + 1, 4) = customer_locations(icustomer, 1);
            D_matrix(icustomer + 1, 5) = customer_locations(icustomer, 2);
    end

    % Makes the S matrix
    S_matrix = transpose(D_matrix) * D_matrix;

    % Calculates the eigenvalues and eigenvectors
    [eigvectors, eigvalues] = eig(S_matrix,C_matrix);

    % Find the smallest eigenvalue and corresponding eigenvector
    smallest_index = 0;
    smallest_eigvalue = inf;
    for     idiagonal = 1 : 6
            if  eigvalues(idiagonal, idiagonal) > 0
                if  eigvalues(idiagonal, idiagonal) < smallest_eigvalue
                    smallest_eigvalue = eigvalues(idiagonal, idiagonal);
                    smallest_index = idiagonal;
                end

            end
    end
    best_eigenvector = eigvectors(:,smallest_index);

    % Apply the scaling
    A_vector = best_eigenvector / sqrt(transpose(best_eigenvector) * C_matrix * best_eigenvector);

    % Makes the x vector to be used in plotting
    x_values = (-70 : .01 : 70);                       % Make the middle value bigger to speed up code

    % Calculates the y vectors
    A = A_vector(3);
    B = (A_vector(2) * x_values) + A_vector(5);
    C = (A_vector(1) * x_values .^2) + (A_vector(4) * x_values) + A_vector(6);
    positive_y_values = (-1 * B + sqrt(B .^2 - 4 * A * C)) / (2 * A);
    negative_y_values = (-1 * B - sqrt(B .^2 - 4 * A * C)) / (2 * A);
    pos_x_values = x_values;
    neg_x_values = x_values;

    % This part is pretty bad so fix if you think of something better
    % Factors out all of the imaginary numbers so the graphing works
    %{
    counter = 0;
    for i = 1:length(positive_y_values)
        if  ~isreal(positive_y_values(i))
            counter = counter + 1;
            pos_imaginary_index(counter) = i;
        end
    end
    counter = 0;
    for i = 1:length(negative_y_values)
        if  ~isreal(negative_y_values(i))
            counter = counter + 1;
            neg_imaginary_index(counter) = i;
        end
    end

    pos_imaginary_index = flip(pos_imaginary_index);
    neg_imaginary_index = flip(neg_imaginary_index);

    for i = 1 : length(pos_imaginary_index)
        positive_y_values(pos_imaginary_index(i)) = [];
        pos_x_values(pos_imaginary_index(i)) = [];
    end
    for i = 1 : length(neg_imaginary_index)
        negative_y_values(neg_imaginary_index(i)) = [];
        neg_x_values(neg_imaginary_index(i)) = [];
    end
    %}

    % Sets some parameters for Newton's Method
    threshold_error = .01;
    max_iterations = 100;
    distances_to_ellipse = zeros(1, customers);

    % Newton's method
    for i = 1 : customers
        error = 1;
        iteration = 0;
        estimate = [customer_locations(i,1), customer_locations(i,2), 0];
        while   (error > threshold_error) && (iteration < max_iterations)
                jh = [2 + (2 * A_vector(1) * estimate(3)), A_vector(2) * estimate(3), (2 * A_vector(1) * estimate(1)) + (A_vector(2) * estimate(2)) + A_vector(4); ...
                     A_vector(2) * estimate(3), 2 + (2 * A_vector(3) *estimate(3)), (A_vector(2) * estimate(1)) + (2 * A_vector(3) * estimate(2)) + A_vector(5); ...
                     (2 * A_vector(1) * estimate(1)) + (A_vector(2) * estimate(2)) + A_vector(4), (A_vector(2) * estimate(1)) + (2 * A_vector(3) * estimate(2)) + A_vector(5), 0];
                
                h = [(2 * estimate(1)) - (2 * customer_locations(i,1)) + (2 * A_vector(1) * estimate(1) * estimate(3)) + (A_vector(2) * estimate(2) * estimate(3)) + (A_vector(4) * estimate(3)); ...
                    (2 * estimate(2)) - (2 * customer_locations(i,2)) + (A_vector(2) * estimate(1) * estimate(3)) + (2 * A_vector(3) * estimate(2) * estimate(3)) + (A_vector(5) * estimate(3)); ...
                    (A_vector(1) * (estimate(1))^2) + (A_vector(2) * estimate(1) * estimate(2)) + (A_vector(3) * (estimate(2))^2) + (A_vector(4) * estimate(1)) + (A_vector(5) * estimate(2)) + A_vector(6)];
                
                next_estimate = estimate + transpose(jh \ (-h));
                error = abs((next_estimate(1) - estimate(1)) + (next_estimate(2) - estimate(2)) + (next_estimate(3) - estimate(3)));
                estimate = next_estimate;
                iteration = iteration + 1;
        end
        distances_to_ellipse(i) = sqrt((estimate(1) - customer_locations(i,1))^2 + (estimate(2) - customer_locations(i,2))^2);
    end

    % Sorts the distances from the ellipse
    [~, customer_indicies] = sort(distances_to_ellipse, "descend");



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Problem with drones using up all the available space that they have



    % Sets up best solution
    best_soln = soln;

    % Calculates the current solution waiting time
    best_waiting_time = calculate_waiting_time(soln, customer_distances, customer_distances_depot);

    % Removes and replaces the customers into the truck route
    max_iterations = 10;
    removal_customer = 0;
    feasible = 1;
    i = 0;

    % Loops until the max iterations is hit or until no feasible position is available
    while feasible == 1 && i < max_iterations
        i = i + 1


        % Changes the default state to not feasible, will be updated to feasible if a better solution is found on this iteration of the while loop
        feasible = 0;

        % Loops over all of the customers in the sorted list 
        for customer = 1 : length(customer_indicies)

            % Selects the customer that is going to be removed from the truck route
            cust_to_remove = customer_indicies(customer);
            
            % Removes the customer from the truck route
            soln_cust_removed = remove_truck_customer(best_soln, cust_to_remove);

            % Loops over all possible places to reinsert the removed customer as a truck customer
            for new_truck_stop = 2 : length(soln_cust_removed.part1) - 1

                % Adds the customer into the truck route at the given stop
                potn_soln = add_truck_customer(soln_cust_removed, cust_to_remove, new_truck_stop);  

                % Checks if the new solution is feasible
                if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

                    % Calculates the waiting time of the new solution
                    potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);

                    % Checks if the new waiting time is better than the best waiting time
                    if  potn_waiting_time < best_waiting_time
                        
                        % Updates the best values to the values of the new solution
                        best_soln = potn_soln;
                        best_waiting_time = potn_waiting_time;
                            
                        % Updates to feasible since a better solution was found
                        feasible = 1;
                    end
                end
            end

            % Loops over all drones, leaving stops, and returning stops to find the best spot to insert the customer as a drone customer
            for drone = 1 : drones
                for leaving_stop = 1 : length(soln_cust_removed.part1) - 1
                    for returning_stop = leaving_stop + 1 : length(soln_cust_removed.part1)
                        
                        % Adds the customer as a drone customer
                        potn_soln = add_drone_customer(soln_cust_removed, cust_to_remove, drone, leaving_stop, returning_stop);

                        % Checks if the solution is feasible
                        if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)
                            
                            % Calculates the waiting time of the new solution
                            potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);

                            if  cust_to_remove == 22
                                fprintf("waiting time: %0.2f, leaving stop: %i, returning stop: %i drone: %i\n", potn_waiting_time, leaving_stop, returning_stop, drone);
                            end


                            % Checks if the new waiting time is better than the best waiting time
                            if  potn_waiting_time < best_waiting_time
                                
                                % Updates the best values to the values of the new solution
                                best_soln = potn_soln;
                                best_waiting_time = potn_waiting_time;

                                % Updates to feasible since a better solution was found
                                feasible = 1;

                                % Keeps track of the customer to remove since he is no longer in the truck route
                                removal_customer = customer;
                                

                                best_soln
                                best_waiting_time
                            end
                        end
                    end
                end
            end
        end

        % Removes the customer from the list if there is one to remove
        if  removal_customer ~= 0
            customer_indicies(removal_customer) = [];
            removal_customer = 0;
        end

    end

    plot_stuff(best_soln, customer_locations)


    % Plots the ellipse
    %{
    figure(2);
    clf;
    hold on;
    plot(pos_x_values, positive_y_values, 'b-');
    plot(neg_x_values, negative_y_values, 'b-');
    scatter(customer_locations(:, 1), customer_locations(:, 2), 'b');
    %}

end

%-----------------------------------------------------------------------%

function[soln_new] = heuristic_two_opt(soln_curr, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % The 2-opt heuristic which switches two customers
    soln_new = soln_curr;

    % Randomly picks the customers
    custA = randi([1, customers]);
    custB = randi([1, customers]);

    % Checks if the customers are the same
    while custA == custB
        custB = randi([1, customers]);
    end

    % Finds where the customers are located
    custA_index = -1;
    custB_index = -1;
    
    % Checks part 1 of the solution for both customers
    for i = 1 : length(soln_curr.part1)
        if  soln_curr.part1(i) == custA
            custA_index = i;
            custA_soln = 1;
        end
        if  soln_curr.part1(i) == custB
            custB_index = i;
            custB_soln = 1;
        end
    end

    % Checks part 2 of the solution for customer A
    if  custA_index == -1
        for i = 1 : length(soln_curr.part2)
            if  soln_curr.part2(i) == custA
                custA_index = i;
                custA_soln = 2;
            end
        end
    end

    % Checks part 2 of the solution for customer B
    if  custB_index == -1
        for i = 1 : length(soln_curr.part2)
            if  soln_curr.part2(i) == custB
                custB_index = i;
                custB_soln = 2;
            end
        end
    end

    % Sets customer B to where customer A was
    if  custA_soln == 1
        soln_new.part1(custA_index) = custB;
    else
        soln_new.part2(custA_index) = custB;
    end

    % Sets customer A to where customer B was
    if  custB_soln == 1
        soln_new.part1(custB_index) = custA;
    else
        soln_new.part2(custB_index) = custA;
    end

    % Checks if the new solution is feasible and calls the drone planner if needed
    if  check_feasible(soln_new, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 0
        soln_new = heuristic_drone_planner(soln_new, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);   
    end
    

end

%-----------------------------------------------------------------------%

function[soln_new] = heuristic_three_opt(soln_curr, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % The 3-opt heuristic which switches two customers
    soln_new = soln_curr;

    % Randomly picks the customers
    custA = randi([1, customers]);
    custB = randi([1, customers]);

    % Checks if the customers are the same
    while custA == custB
        custB = randi([1, customers]);
    end

    % Generates the third customer and makes sure it is different than the others
    custC = randi([1, customers]);
    while (custC == custA) || (custC == custB)
        custC = randi([1, customers]);
    end

    % Finds where the customers are located
    custA_index = -1;
    custB_index = -1;
    custC_index = -1;
    
    % Checks part 1 of the solution for both customers
    for i = 1 : length(soln_curr.part1)
        if  soln_curr.part1(i) == custA
            custA_index = i;
            custA_soln = 1;
        end
        if  soln_curr.part1(i) == custB
            custB_index = i;
            custB_soln = 1;
        end
        if  soln_curr.part1(i) == custC
            custC_index = i;
            custC_soln = 1;
        end
    end

    % Checks part 2 of the solution for customer A
    if  custA_index == -1
        for i = 1 : length(soln_curr.part2)
            if  soln_curr.part2(i) == custA
                custA_index = i;
                custA_soln = 2;
            end
        end
    end

    % Checks part 2 of the solution for customer B
    if  custB_index == -1
        for i = 1 : length(soln_curr.part2)
            if  soln_curr.part2(i) == custB
                custB_index = i;
                custB_soln = 2;
            end
        end
    end

    % Checks part 2 of the solution for customer C
    if  custC_index == -1
        for i = 1 : length(soln_curr.part2)
            if  soln_curr.part2(i) == custC
                custC_index = i;
                custC_soln = 2;
            end
        end
    end

    % Sets customer C to where customer A was
    if  custA_soln == 1
        soln_new.part1(custA_index) = custC;
    else
        soln_new.part2(custA_index) = custC;
    end

    % Sets customer A to where customer B was
    if  custB_soln == 1
        soln_new.part1(custB_index) = custA;
    else
        soln_new.part2(custB_index) = custA;
    end

    % Sets customer B to where customer C was
    if  custC_soln == 1
        soln_new.part1(custC_index) = custB;
    else
        soln_new.part2(custC_index) = custB;
    end

    % Checks if the new solution is feasible and calls the drone planner if needed
    if  check_feasible(soln_new, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 0
        soln_new = heuristic_drone_planner(soln_new, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
    end
    

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_greedy(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % Makes the list of truck stops that are not leaving or returning stops
    list_of_cust_to_remove = soln.part1;
    list_of_cust_to_remove(1) = [];
    list_of_cust_to_remove(length(list_of_cust_to_remove)) = [];

    % Checks the leaving stops and removes any stops where a drone leaves
    for i = 1 : length(soln.part3)
        if  soln.part3(i) == -1
            continue
        else
            for i_truck = length(list_of_cust_to_remove) : -1 : 1 %%%%%%%%%%%%% This might be a problem
                if  soln.part1(soln.part3(i)) == list_of_cust_to_remove(i_truck)
                    list_of_cust_to_remove(i_truck) = [];
                end
            end
        end
    end

    % Checks the returning stops and removes any stops where a drone returns
    for i = 1 : length(soln.part4)
        if  soln.part4(i) == -1
            continue
        else
            for i_truck = length(list_of_cust_to_remove) : -1 : 1
                if  soln.part1(soln.part4(i)) == list_of_cust_to_remove(i_truck)
                    list_of_cust_to_remove(i_truck) = [];
                end
            end
        end
    end

    % Adds the drones to the list of possible customers to remove
    for i = 1 : length(soln.part2)
        if  soln.part2(i) ~= -1
            list_of_cust_to_remove(length(list_of_cust_to_remove) + 1) = soln.part2(i);
        end
    end

    % Picks a customer at random from the list
    rand_index = randi([1, length(list_of_cust_to_remove)]);
    cust_to_remove = list_of_cust_to_remove(rand_index);

    %%%%%%%%%%%%%%%%%%%%% random set up stuff
    % Sets up current solution
    potn_soln.part1 = soln.part1;
    potn_soln.part2 = soln.part2;
    potn_soln.part3 = soln.part3;
    potn_soln.part4 = soln.part4;

    % Sets up the best solution
    best_soln.part1 = soln.part1;
    best_soln.part2 = soln.part2;
    best_soln.part3 = soln.part3;
    best_soln.part4 = soln.part4;
    
    % Calculates the current solution waiting time
    current_waiting_time = calculate_waiting_time(soln, customer_distances, customer_distances_depot);
    best_waiting_time = current_waiting_time;

    % * stuff from the elliptical algorithm
    soln_cust_removed = remove_customer(soln, cust_to_remove);
    for truck_stop = 2 : length(potn_soln.part1)
        potn_soln = add_truck_customer(soln_cust_removed, cust_to_remove, truck_stop);
        if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 1
            potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);
            if  potn_waiting_time < best_waiting_time
                best_soln = potn_soln;
                best_waiting_time = potn_waiting_time;
            end

            %fprintf('customer = %d, truck stop = %d, waiting time = %f, best waiting = %f\n', rand_cust, truck_stop, potn_waiting_time, best_waiting_time)



        end
    end

    % Now it does the drone routes
    for drone = 1 : drones
        for leaving_stop = 1 : length(soln_cust_removed.part1) - 1
            for returning_stop = leaving_stop + 1 : length(soln_cust_removed.part1)
                potn_soln = add_drone_customer(soln_cust_removed, cust_to_remove, drone, leaving_stop, returning_stop);
                if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 1
                    potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);
                    if  potn_waiting_time < best_waiting_time
                        best_soln = potn_soln;
                        best_waiting_time = potn_waiting_time;
                    end

                    %fprintf('customer = %d, drone = %d, leaving stop = %d, returning stop = %d waiting time = %f, best waiting = %f\n', rand_cust, ...
                        %drone, leaving_stop, returning_stop, potn_waiting_time, best_waiting_time)

                end
            end
        end
    end

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_origin_destination(soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % Origin Destination heuristic

    % Makes the boolean vector which tells us if the truck customer has been added to the list already
    cust_already_in_list = zeros(1, customers);
    index_for_final_list = 0;

    % Checks part 3 of the solution and adds the truck customers that the drones leaves from into the rendevou list
    for i = 1 : length(soln.part3)
        if  soln.part3(i) ~= -1
            truck_cust = soln.part1(soln.part3(i));
            if  truck_cust ~= 0 
                if  cust_already_in_list(truck_cust) == 0
                    cust_already_in_list(truck_cust) = 1;
                    index_for_final_list = index_for_final_list + 1;
                    list_truck_cust(index_for_final_list) = truck_cust;
                end
            end
        end
    end

    % Now does the same thing but for part 4 of the solution
    for i = 1 : length(soln.part4)
        if  soln.part4(i) ~= -1
            truck_cust = soln.part1(soln.part4(i));
            if  truck_cust ~= 0 
                if  cust_already_in_list(truck_cust) == 0
                    cust_already_in_list(truck_cust) = 1;
                    index_for_final_list = index_for_final_list + 1;
                    list_truck_cust(index_for_final_list) = truck_cust;
                end
            end
        end
    end


    % Selects a customer at random from the list
    rand_index = randi([1, length(list_truck_cust)]);
    cust_to_remove = list_truck_cust(rand_index);


    %%%%%%%%%%%%%%%%%%%%% random set up stuff
    % Sets up current solution
    potn_soln.part1 = soln.part1;
    potn_soln.part2 = soln.part2;
    potn_soln.part3 = soln.part3;
    potn_soln.part4 = soln.part4;

    % Sets up the best solution
    best_soln.part1 = soln.part1;
    best_soln.part2 = soln.part2;
    best_soln.part3 = soln.part3;
    best_soln.part4 = soln.part4;
    
    % Calculates the current solution waiting time
    current_waiting_time = calculate_waiting_time(soln, customer_distances, customer_distances_depot);
    best_waiting_time = current_waiting_time;

    % * stuff from the elliptical algorithm but only the truck stuff
    soln_cust_removed = remove_customer(soln, cust_to_remove);
    for truck_stop = 2 : length(potn_soln.part1)
        potn_soln = add_truck_customer(soln_cust_removed, cust_to_remove, truck_stop);
        potn_soln = heuristic_drone_planner(potn_soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);
        if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 1
            potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);
            if  potn_waiting_time < best_waiting_time
                best_soln = potn_soln;
                best_waiting_time = potn_waiting_time;
            end
        end
    end



end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_drone_planner(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)


    best_soln = soln;
    for iteration = 1 : 10
        % Makes the big matrix that has all the possible routes (Pj)
        possible_routes = [];
        row_counter = 0;
        new_soln = soln;
    
        icustomer = 1;
        for idrone = 1 : drones
            while (icustomer <= length(soln.part2)) && (soln.part2(icustomer) ~= -1)
                for ileaving = 1 : length(soln.part1) - 1
                    for ireturning = ileaving + 1 : length(soln.part1)
                        row_counter = row_counter + 1;
                        possible_routes(row_counter,:) = [idrone, soln.part2(icustomer), soln.part1(ileaving), soln.part1(ireturning)];
                    end
                end
                icustomer = icustomer + 1;
            end
            icustomer = icustomer + 1;
        end
    
    
        bdone = 0;
        idrone = 1;
        icust = 1;
        % Picks the first route for the first customers
        while (idrone <= drones) && (bdone == 0)
            while (icust ~= length(soln.part2)) && bdone == 0
                if  soln.part2(icust) == -1
                    icust = icust + 1;
                    continue
                end
    
                num_routes = 1;
                if  possible_routes(num_routes, 2) == soln.part2(icust)
                    routes_done = 0;
                else
                    bdone = 1;
                end
                
                starting_row = num_routes;
                while routes_done == 0
                    num_routes = num_routes + 1;
                    if  soln.part2(icust) ~= possible_routes(num_routes, 2)
                        routes_done = 1;
                    end
                end
    
                row_picked = randi([starting_row, num_routes]);
                route_picked = [possible_routes(row_picked, 3), possible_routes(row_picked, 4)];
                
                [pos_routes, ~] = size(possible_routes);
                
                route = num_routes;
    
                while route <= pos_routes
                    if  (possible_routes(route, 3) < route_picked(1)) && (possible_routes(route, 4) <= route_picked(1))
                        bokay = 1;
                    elseif (possible_routes(route, 3) >= route_picked(2)) && (possible_routes(route, 4) > route_picked(2))
                        bokay = 1;
                    else
                        bokay = 0;
                    end
    
                    if  bokay == 0
                        possible_routes(route, :) = [];
                        [pos_routes, ~] = size(possible_routes);
                    else
                        route = route + 1;
                    end
                end
    
                % Changes the customer numbers from the pj matrix to indicies
                for i  = 1 : length(soln.part1) - 1
                    if  soln.part1(i) == route_picked(1)
                        index1 = i;
                    end
                end
                for i  = 2 : length(soln.part1)
                    if soln.part1(i) == route_picked(2)
                        index2 = i;
                    end
                end
                new_soln.part3(icust) = index1;
                new_soln.part4(icust) = index2;
                icust = icust + 1;
            end
            idrone = idrone + 1;
        end

        % Sorts the solution before putting it in the feasibility function
        position = 1;
        starting_position = 1;
        for drone = 1 : drones
            if  position > length(new_soln.part3)
                break
            end
            while new_soln.part3(position) ~= -1 && position ~= length(new_soln.part3)
                position = position + 1;
            end

            % Checks if there are no customer assigned to that drone
            if  starting_position == position
                starting_position = starting_position + 1;
                position = position + 1;
            else
                position = position - 1;                % Makes it so that the position is not on a -1
                [sorted_list, indicies_of_sort] = sort(new_soln.part3(starting_position : position));
                new_soln.part3(starting_position : position) = sorted_list;
                temp_part2 = new_soln.part2(starting_position : position);
                temp_part4 = new_soln.part4(starting_position : position);
                for i = 1 : length(temp_part2)
                    new_soln.part2(starting_position + i - 1) = temp_part2(indicies_of_sort(i));
                    new_soln.part4(starting_position + i - 1) = temp_part4(indicies_of_sort(i));
                end
                position = position + 2;
                starting_position = position;
            end
        end

        if  check_feasible(best_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)
            if  check_feasible(new_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) ...
                && (calculate_waiting_time(new_soln, customer_distances, customer_distances_depot) < calculate_waiting_time(best_soln, customer_distances, customer_distances_depot))
                best_soln = new_soln;
            end 
        else
            best_soln = new_soln;
        end

        
    end

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_general_assignment(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % General Assignment heuristic

    % This list making process was copied from Greedy
    % Makes the list of truck stops that are not leaving or returning stops
    list_of_cust_to_remove = soln.part1;
    list_of_cust_to_remove(1) = [];
    list_of_cust_to_remove(length(list_of_cust_to_remove)) = [];

    % Checks the leaving stops and removes any stops where a drone leaves
    for i = 1 : length(soln.part3)
        if  soln.part3(i) == -1
            continue
        else
            for i_truck = length(list_of_cust_to_remove) : -1 : 1 %%%%%%%%%%%%% This might be a problem
                if  soln.part1(soln.part3(i)) == list_of_cust_to_remove(i_truck)
                    list_of_cust_to_remove(i_truck) = [];
                end
            end
        end
    end

    % Checks the returning stops and removes any stops where a drone returns
    for i = 1 : length(soln.part4)
        if  soln.part4(i) == -1
            continue
        else
            for i_truck = length(list_of_cust_to_remove) : -1 : 1
                if  soln.part1(soln.part4(i)) == list_of_cust_to_remove(i_truck)
                    list_of_cust_to_remove(i_truck) = [];
                end
            end
        end
    end

    % Adds the drones to the list of possible customers to remove
    for i = 1 : length(soln.part2)
        if  soln.part2(i) ~= -1
            list_of_cust_to_remove(length(list_of_cust_to_remove) + 1) = soln.part2(i);
        end
    end

    % Selecting the random integer for the number of customers to remove (N)
    num_cust_to_remove = randi([2, 5]);

    % Generates a list of random indicies with the length of customers that we need to remove
    indicies_to_remove = sort(randperm(length(list_of_cust_to_remove), num_cust_to_remove), 'descend');

    % Removes all selected customers from the solution
    soln_cust_removed = soln;
    for i = 1 : num_cust_to_remove
        soln_cust_removed = remove_customer(soln_cust_removed, list_of_cust_to_remove(indicies_to_remove(i)));
        b_customers_in_route(list_of_cust_to_remove(indicies_to_remove(i))) = 0;
    end

    
    % Sets up current solution
        potn_soln = soln_cust_removed;
    
        % Sets up the best solution
        best_soln = soln_cust_removed;

    for i = 1 : num_cust_to_remove
        % Copy paste * stuff from greedy
        

        best_waiting_time = inf;
    
        % * stuff from the elliptical assignment but i'm probably going to change some stuff to fit this
        for truck_stop = 2 : length(potn_soln.part1)
            potn_soln = add_truck_customer(soln_cust_removed, list_of_cust_to_remove(indicies_to_remove(i)), truck_stop);
            b_customers_in_route(list_of_cust_to_remove(indicies_to_remove(i))) = 1;
            if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 1
                potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);
                if  potn_waiting_time < best_waiting_time
                    best_soln = potn_soln;
                    best_waiting_time = potn_waiting_time;
                end
    
                %fprintf('customer = %d, truck stop = %d, waiting time = %f, best waiting = %f\n', rand_cust, truck_stop, potn_waiting_time, best_waiting_time)
    
            end
        end
    
        % Now it does the drone routes
        for drone = 1 : drones
            for leaving_stop = 1 : length(soln_cust_removed.part1) - 1
                for returning_stop = leaving_stop + 1 : length(soln_cust_removed.part1)
                    potn_soln = add_drone_customer(soln_cust_removed, list_of_cust_to_remove(indicies_to_remove(i)), drone, leaving_stop, returning_stop);
                    b_customers_in_route(list_of_cust_to_remove(indicies_to_remove(i))) = 1;
                    if  check_feasible(potn_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 1
                        potn_waiting_time = calculate_waiting_time(potn_soln, customer_distances, customer_distances_depot);
                        if  potn_waiting_time < best_waiting_time
                            best_soln = potn_soln;
                            best_waiting_time = potn_waiting_time;
                        end
    
                        %fprintf('customer = %d, drone = %d, leaving stop = %d, returning stop = %d waiting time = %f, best waiting = %f\n', rand_cust, ...
                            %drone, leaving_stop, returning_stop, potn_waiting_time, best_waiting_time)
    
                    end
                end
            end
        end
        soln_cust_removed = best_soln;
    end
    
    % Makes sure the best solution is feasible and if not it outputs the input solution
    if  check_feasible(best_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) == 0
        best_soln = soln;
    end

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_wild_change(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % Wild Change Heuristic

    % Saves the output solution as the input solution
    best_soln = soln;

    % Generates the random number from 2 to 5
    num_cust_to_remove = randi([2, 5]);

    % Picks the customer numbers to remove
    customers_to_remove = sort(randperm(customers, num_cust_to_remove), 'descend');

    % Removes the customers from the route
    soln_cust_removed = soln;
    for i = 1 : num_cust_to_remove
        soln_cust_removed = remove_customer(soln_cust_removed, customers_to_remove(i));
    end

    

    % Assigns each customer to either a drone or the truck route
    for i = 1 : num_cust_to_remove
        
        % Picks a number that is the number of drones + 1 where 1 is the truck and any other number corresponds to a drone
        rand_num = randi([1, drones + 1]);

        if  rand_num == 1
            
            % Picks the gap between the customers where the new customer will be inserted in the truck route
            stop_to_insert = randi([2, length(soln_cust_removed.part1)]);

            % Inserts the customer as a truck customer
            soln_cust_removed = add_truck_customer(soln_cust_removed, customers_to_remove(i), stop_to_insert);

        else
            % Inserts the customer as a drone customer into the first slot of that drone
            soln_cust_removed = add_drone_customer(soln_cust_removed, customers_to_remove(i), rand_num - 1, 100 * i, 100 * i);  % 100 * i is to give the functions something to insert for parts 3 and 4 and also so it is not inserting the same value bc that might break it
        end
    end
    
    soln_prime = soln_cust_removed;

    % Runs the drone planner on the new solution
    soln_prime = heuristic_drone_planner(soln_prime, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route);

    % If the new solution is feasible it saves it as the best solution
    if  check_feasible(soln_prime, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)
        best_soln = soln_prime;
    end

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_truck_reorder(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations)

    % Reorders the truck customers so that they resemble a circular truck route

    % Sets up the new solution and best solution
    best_soln = soln;
    soln_prime = soln;

    % Initlizes the truck customers angle's vector
    cust_angles = zeros(1, length(soln.part1) - 2);

    % Changes the cartestian coordinates to an angle theta
    for i = 2 : length(soln.part1) - 1
        cust_angles(i - 1) = atan2(customer_locations(soln.part1(i), 1), customer_locations(soln.part1(i), 2));
    end

    % Sorts the angles of the customer locations from lowest (-pi) to highest (pi)
    [~, indicies] = sort(cust_angles);

    % Finds the index of the first truck stop
    for i = 1 : length(indicies)
        if  indicies(i) == 1
            starting_index = i;
        end
    end

    % Changes the truck route based on angle
    for i = 3 : length(soln.part1) - 1     % We are skipping the first depot, leaving the first customer alone, and skipping the last depot
        soln_prime.part1(i) = soln.part1(indicies(mod(starting_index + i - 3, length(indicies)) + 1) + 1);
    end

    % If the new solution is feasible and better than the current it saves it as the best solution
    if  check_feasible(soln_prime, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) ...
        && (calculate_waiting_time(soln_prime, customer_distances, customer_distances_depot) < calculate_waiting_time(best_soln, customer_distances, customer_distances_depot))
        best_soln = soln_prime;
    end

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_truck_outlier_removal(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)
    
    % Moves a truck customer that is far away from the depot to be a drone customer

    best_soln = soln;

    % Find the radius that will act as the threshold for being too far away from the depot
    distance_threshold = 30;

    % Sets up the customer to remove variables
    cust_to_remove = -1;
    farthest_distance = 0;

    % Checks if any truck customers are beyond the threshold, if so, pickes the farthest one
    for i = 2 : length(soln.part1) - 1
        if  (customer_distances_depot(soln.part1(i), 1) > distance_threshold) && (customer_distances_depot(soln.part1(i), 1) > farthest_distance)
            cust_to_remove = soln.part1(i);
            farthest_distance = customer_distances_depot(soln.part1(i), 1);
        end
    end

    % Makes sure at least one customer was outside the threshold
    if cust_to_remove ~= -1

        % Remove the chosen customer
        soln_cust_removed = remove_truck_customer(soln, cust_to_remove);
    
        % Finds where to put the customer as a drone customer  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Finds the drone delivering to the least amount of people
        delivery_total = zeros(1, drones);
        
        i = 1;
        for drone = 1 : drones
            while (i <= length(soln_cust_removed.part2)) && (soln_cust_removed.part2(i) ~= -1)
                delivery_total(drone) = delivery_total(drone) + 1;
                i = i + 1;
            end
            i = i + 1;
        end

        [~, drone_to_insert] = max(delivery_total);

        % Compresses all of the long routes of the chosen drone
        soln_cust_removed = drone_route_compressor(soln_cust_removed, drone_to_insert, drones, customer_distances, customer_distances_depot);

        % Finds a gap in the chosen drone's route to put in the new drone customer's route
        i = 1;
        leaving_stop = -1;
        returning_stop = -1;

        % Gets us to the right drone and also makes sure we are not at the end of the list or at a negative 1
        for drone = 1 : drones
            while (i < length(soln_cust_removed.part2)) && (soln_cust_removed.part2(i) ~= -1)
                if  drone == drone_to_insert

                    % Looks at the returning stop of one drone cust and the leaving stop of the next one to see if there is at least a 2 wide gap between them
                    if  soln_cust_removed.part4(i) <= soln_cust_removed.part3(i + 1) - 2 
                        leaving_stop = soln_cust_removed.part4(i);
                        returning_stop = soln_cust_removed.part3(i + 1);
                    end
                end
                i = i + 1;
            end
            i = i + 1;
        end

        
        % Only tries to find a better solution if there is a gap in the drone route for another drone customer
        if  (returning_stop ~= -1) && (leaving_stop ~= -1)
            % Adds the customer in as a drone customer
            new_soln = add_drone_customer(soln_cust_removed, cust_to_remove, drone_to_insert, leaving_stop, returning_stop);

            %soln
            %calculate_waiting_time(soln, customer_distances, customer_distances_depot)
            %new_soln
            %calculate_waiting_time(new_soln, customer_distances, customer_distances_depot)
            %plot_stuff(soln, customer_locations);


            % Checks if new solution is feasible and better than the input solution
            if  check_feasible(new_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) && (calculate_waiting_time(new_soln, customer_distances, customer_distances_depot) < calculate_waiting_time(soln, customer_distances, customer_distances_depot))
                best_soln = new_soln;
                %fprintf('Better\n');
            else
                %fprintf('Worse\n');
            end
        else
            %fprintf('Could not find opening\n');
        end
    
        
    end

end

%-----------------------------------------------------------------------%

function[soln] = drone_route_compressor(soln, drone_to_compress, drones, customer_distances, customer_distances_depot)

    % Compresses one of the drones routes in order to make some of the longer drone trips shorter

    i = 1;

%     fprintf('Compressor');
%     calculate_waiting_time(soln, customer_distances, customer_distances_depot)
%     soln
%     print_matrix(customer_distances_depot);

    % Loops over all the drone to find the one we are looking for
    for drone = 1 : drones

        % Checks if we are at the end of the list or looking at a negative one
        while (i < length(soln.part2)) && (soln.part2(i) ~= -1)

            % Checks if we are on the drone that we are looking for
            if  drone == drone_to_compress

                % Looks at a customers drone route to see if there is at least a 4 wide gap in it
                if  soln.part3(i) <= soln.part4(i) - 4
                    gap = soln.part4(i) - soln.part3(i);

                    % Shortens cuts all the drone routes that are too long in half
                    soln.part4(i) = floor(soln.part4(i) - gap/2);

                end
            end
            i = i + 1;
        end
        i = i + 1;
    end

    %calculate_waiting_time(soln, customer_distances, customer_distances_depot)

end

%-----------------------------------------------------------------------%

function[best_soln] = heuristic_drone_to_truck(soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations)

    % Changes a drone customer that is close to the depot to be a truck customer

    best_soln = soln;

    % Find the radius that will act as the threshold for being too close to the depot
    distance_threshold = 20;

    % Sets up the customer to remove variables
    cust_to_remove = -1;
    closest_distance = inf;

    % Checks if any drone customers are within the threshold, if so, pickes the closest one
    for i = 1 : length(soln.part2)
        if  soln.part2(i) ~= -1
            if  (customer_distances_depot(soln.part2(i), 1) < distance_threshold) && (customer_distances_depot(soln.part2(i), 1) < closest_distance)
                cust_to_remove = soln.part2(i);
                closest_distance = customer_distances_depot(soln.part2(i), 1);
            end
        end
    end

    % Makes sure at least one customer was inside the threshold
    if cust_to_remove ~= -1

        % Remove the chosen customer
        soln_cust_removed = remove_drone_customer(soln, cust_to_remove);
    
        % Adds the customer in as a truck customer
        new_soln = add_truck_customer(soln_cust_removed, cust_to_remove, length(soln.part1));

        % Calls the truck reorder heuristic to put the new truck stop in a better location
        new_soln = heuristic_truck_reorder(new_soln, drones, customers, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route, customer_locations);
    
        % Checks if new solution is feasible and better than the input solution
        if  check_feasible(new_soln, customers, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route) && (calculate_waiting_time(new_soln, customer_distances, customer_distances_depot) < calculate_waiting_time(soln, customer_distances, customer_distances_depot))
            best_soln = new_soln;
        end
    end

end

%-----------------------------------------------------------------------%

function[total_waiting_time] = calculate_waiting_time(soln, customer_distances, customer_distances_depot)

    % Calculates the waiting time for a given solution

    % Initilizes lists for truck and drone waiting times
    truck_waiting_time = zeros(1, length(soln.part1) - 1);
    drone_waiting_time = zeros(1, length(soln.part2));

    % Random variables that I might need
    current_waiting_time = 0;        % Used for when the drone gets to the stop after the truck.


    for truck_stop = 1 : length(soln.part1)

        % This is for the truck customer's waiting time
        if  truck_stop == 1 || truck_stop == length(soln.part1)

        elseif soln.part1(truck_stop - 1) == 0
            truck_waiting_time(truck_stop - 1) = customer_distances_depot(1, soln.part1(truck_stop + 1));
            current_waiting_time = current_waiting_time + customer_distances_depot(1, soln.part1(truck_stop + 1));
        else
            truck_waiting_time(truck_stop - 1) = current_waiting_time + customer_distances(soln.part1(truck_stop - 1), soln.part1(truck_stop));
            current_waiting_time = current_waiting_time + customer_distances(soln.part1(truck_stop - 1), soln.part1(truck_stop));
        end

        % Now this will start looking at the drones and only really does something when the drone returns to the truck at the current truck stop
        for drone_return = 1 : length(soln.part4)

            % Skips this iteration if the value is the seperator for the drones
            if  soln.part4(drone_return) == -1
                continue
            elseif truck_stop == soln.part4(drone_return)

                % Drone waiting time if the drone left the truck at the depot
                if  soln.part3(drone_return) == 1
                    drone_waiting_time(drone_return) = customer_distances_depot(1, soln.part2(drone_return) + 1) / 1.5;

                    % This is to check if the drone or the truck gets to the next stop first and updates the current waiting time accordingly but this one is for starting at the depot
                    truck_travel_stops = (1 : truck_stop);
                    truck_travel_time = customer_distances_depot(1, soln.part1(2) + 1);
                    for truck_travel_stop = 2 : length(truck_travel_stops) - 1
                        if  soln.part1(truck_travel_stops(truck_travel_stop) + 1) == 0
                            truck_travel_time = truck_travel_time + customer_distances_depot(soln.part1(truck_travel_stops(truck_travel_stop)) + 1, 1);
                        else
                            truck_travel_time = truck_travel_time + customer_distances(soln.part1(truck_travel_stops(truck_travel_stop)), soln.part1(truck_travel_stops(truck_travel_stop) + 1));
                        end
                    end
                    
                    % Checks if the drone returns at the depot and then calculates the time it takes to do its route
                    if  soln.part4(drone_return) ~= length(soln.part1)
                        drone_travel_time = (customer_distances_depot(1, soln.part2(drone_return) + 1) + customer_distances(soln.part2(drone_return), soln.part1(soln.part4(drone_return)))) / 1.5;
                    else
                        drone_travel_time = (customer_distances_depot(1, soln.part2(drone_return) + 1) + customer_distances_depot(soln.part2(drone_return) + 1, 1)) / 1.5;
                    end
                    
                    % Compares the truck and drone times to see which one gets to the truck later
                    if   truck_travel_time < drone_travel_time
                        current_waiting_time = drone_travel_time;
                    end

                    % Drone waiting time if the drone left the truck anywhere but the depot
                else
                    drone_waiting_time(drone_return) = truck_waiting_time(soln.part3(drone_return)) + (customer_distances(soln.part3(drone_return), soln.part2(drone_return)) / 1.5);

                    % Only checks if the drone or the truck gets the stop first if the drone does not come back at the depot
                    if  soln.part4(drone_return) ~= length(soln.part1)

                        % This is to check if the drone or the truck gets to the next stop first and updates the current waiting time accordingly
                        truck_travel_stops = (soln.part3(drone_return) : truck_stop);
                        truck_travel_time = 0;
                        for truck_travel_stop = 1 : length(truck_travel_stops) - 1
                            truck_travel_time = truck_travel_time + customer_distances(soln.part1(truck_travel_stops(truck_travel_stop)), soln.part1(truck_travel_stops(truck_travel_stop) + 1));
                        end
                        drone_travel_time = (customer_distances(soln.part1(soln.part3(drone_return)), soln.part2(drone_return)) + customer_distances(soln.part2(drone_return), soln.part1(soln.part4(drone_return)))) / 1.5;
                        if  (truck_travel_time < drone_travel_time) && (current_waiting_time < (truck_waiting_time(soln.part3(drone_return) - 1) + drone_travel_time))
                            current_waiting_time = truck_waiting_time(soln.part3(drone_return) - 1) + drone_travel_time;
                        end
                    end
                end
            end
        end
    end
    total_waiting_time = sum(truck_waiting_time) + sum(drone_waiting_time);

end

%-----------------------------------------------------------------------%

function[soln, customer_to_remove_index] = remove_truck_customer(soln, customer)
    
    % Removes a truck customer from the solution

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Look here for error with eca changing leaving/returning stops
   
    % Finds the customer to remove
    for cust_index = 1 : length(soln.part1)
        if  customer == soln.part1(cust_index)
            customer_to_remove_index = cust_index;
            break
        else
            customer_to_remove_index = -1;
        end
    end
    
    % Checks if the customer actually got removed from the truck route
    if  customer_to_remove_index ~= -1
        
        % Removes the customer from the solution
        soln.part1(customer_to_remove_index) = [];

        % Fixes the drones
        for drone_index = 1 : length(soln.part3)
            if  soln.part3(drone_index) == -1
                continue
            else
                if  soln.part3(drone_index) > customer_to_remove_index
                    soln.part3(drone_index) = soln.part3(drone_index) - 1;
                end
                if  soln.part4(drone_index) > customer_to_remove_index
                    soln.part4(drone_index) = soln.part4(drone_index) - 1;
                end
            end
        end
        %%%%%%%%%%%%%%%%%% Check for out and back trips and then change where the next drone leaves from if there is an issue
    end

end

%-----------------------------------------------------------------------%

function[soln] = add_truck_customer(soln, customer, stop)

    % Adds the customer to the given stop


    new_list = zeros(1, length(soln.part1) + 1);

    % Sets the new list as the same as the solution for the stops below the stop
    for cust_index = 1 : stop - 1
        new_list(cust_index) = soln.part1(cust_index);
    end

    % Shifts every stop above the given stop up by 1
    for cust_index = stop : length(soln.part1)
        new_list(cust_index + 1) = soln.part1(cust_index);
    end
    
    % Shifts the indexes of the drones at or above the given stop up by one
    for drone_index = 1 : length(soln.part3)
        if  soln.part3(drone_index) >= stop
            soln.part3(drone_index) = soln.part3(drone_index) + 1;                         
        end
        if  soln.part4(drone_index) >= stop
            soln.part4(drone_index) = soln.part4(drone_index) + 1;                         
        end
    end
   
    % Adds the customer to the new truck route
    new_list(stop) = customer;

    % Sets the new list back to the solution
    soln.part1 = new_list;


end

%-----------------------------------------------------------------------%

function[soln] = add_drone_customer(soln, customer, drone, leaving_stop, returning_stop)
    
    % Adds a drone customer to the route


    % Sets the drone counter to 1 and counter for the index to 1
    drone_count = 1;
    drone_index = 1;

    % Looks for the requested drone and finds the index to the customer after the -1
    while drone_count ~= drone
        if  soln.part3(drone_index) == -1
            drone_count = drone_count + 1;
        end
        drone_index = drone_index + 1;
    end
    
    % Looks for where to insert the new customer to keep part 3 and 4 order unless it is to be inserted as the last element in the list
    if  drone_index <= length(soln.part3)
        while (drone_index <= length(soln.part3)) && (leaving_stop > soln.part3(drone_index)) && (soln.part3(drone_index) ~= -1)
            drone_index = drone_index + 1;
        end
    end 
    
    % Makes new variables to shift the drone stuff up by one
    temp_soln.part1 = soln.part1;
    temp_soln.part2 = zeros(1, length(soln.part2) + 1);
    temp_soln.part3 = zeros(1, length(soln.part3) + 1);
    temp_soln.part4 = zeros(1, length(soln.part4) + 1);

    % Puts all of the indexes below the given stop as the same
    if  drone_index ~= 1
        for i = 1 : drone_index - 1
            temp_soln.part2(i) = soln.part2(i);
            temp_soln.part3(i) = soln.part3(i);
            temp_soln.part4(i) = soln.part4(i);
        end
    end
    
    % Shifts the drone columns up by one to make space for the new drone customer
    for i = drone_index : length(soln.part2)
        temp_soln.part2(i + 1) = soln.part2(i);
        temp_soln.part3(i + 1) = soln.part3(i);
        temp_soln.part4(i + 1) = soln.part4(i);
    end
    
    % Insert the new customer into the drone route
    temp_soln.part2(drone_index) = customer;
    temp_soln.part3(drone_index) = leaving_stop;
    temp_soln.part4(drone_index) = returning_stop;
    soln = temp_soln;


end

%-----------------------------------------------------------------------%

function[soln] = remove_drone_customer(soln, customer)

    % Removes a drone customer from the solution

    % Finds the index of the customer in the solution
    for i = 1 : length(soln.part2)
        if  soln.part2(i) == customer
            customer_to_remove_index = i;
        end
    end

    % Removes the customer from the list
    soln.part2(customer_to_remove_index) = [];
    soln.part3(customer_to_remove_index) = [];
    soln.part4(customer_to_remove_index) = [];

end

%-----------------------------------------------------------------------%

function[soln] = remove_customer(soln, customer)
    
    % Determines whether the customer to remove is a drone or truck customer and calls the respective function to remove it
    
    % variable for if the customer is a truck customer
    truck_customer = 0;

    for i = 1 : length(soln.part1)
        if  customer == soln.part1(i)
            [soln, ~] = remove_truck_customer(soln, customer);
            truck_customer = 1;
            break
        end
    end

    if  truck_customer == 0
        soln = remove_drone_customer(soln, customer);
    end

end

%-----------------------------------------------------------------------%

function[feasible] = check_feasible(soln, ~, drones, customer_distances, customer_distances_depot, max_flight_time, b_customers_in_route)

    % Checks if a solution is feasible

    % Sets up the feasible condition
    feasible = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Work on this
    % Checks if all of the customers are being delivered to
    for customer = 1 : length(b_customers_in_route) 
        if  b_customers_in_route(customer) == 1
            found = 0;
            for i = 1 : length(soln.part1)
                if  customer == soln.part1(i)
                    found = 1;
                end    
            end
            for i = 1 : length(soln.part2)
                if  customer == soln.part2(i)
                    found = 1;
                end
            end
            if  found == 0
                feasible = 0;
                %fprintf('Customer not being delievered to\n')
            end
        end
    end


    % Checks if the truck stops and starts at the depot
    if  (soln.part1(1) ~= 0) || (soln.part1(length(soln.part1)) ~= 0)
        feasible = 0;
        %fprintf('Truck not starting or stopping at depot\n')
    end
    
    % Makes sure that the drones return to a stop later from the one they leave at
    if  feasible == 1
        for i = 1 : length(soln.part3)
            if  ((soln.part4(i) <= soln.part3(i)) && (soln.part3(i) ~= -1) && (soln.part4(i) ~= -1)) %|| soln.part4(i) == length(soln.part1)
                feasible = 0;
                %fprintf('drones return to a stop later from the one they leave at\n')
                break
            end

            % Makes sure there are no overlapping trips
            if  i == length(soln.part3)
                continue
            else
                if  (soln.part4(i) > soln.part3(i + 1)) && (soln.part4(i) ~= -1) && (soln.part3(i + 1) ~= -1)
                    feasible = 0;
                    %fprintf('there are overlapping trips\n')
                    %soln
                    break
                end
            end
        
            % Makes sure that the drone flight times are below the maximum amount
            if  soln.part3(i) ~= -1
                if  (soln.part3(i) == 1) && (soln.part4(i) == length(soln.part1))
                    flight_time = customer_distances_depot(1, soln.part2(i) + 1) + customer_distances_depot(soln.part2(i) + 1, soln.part1(soln.part4(i)) + 1);
                elseif soln.part3(i) == 1 && soln.part4(i) ~= length(soln.part1)
                    flight_time = customer_distances_depot(1, soln.part2(i) + 1) + customer_distances(soln.part2(i), soln.part1(soln.part4(i)));
                elseif soln.part4(i) == length(soln.part1)
                    flight_time = customer_distances(soln.part1(soln.part3(i)), soln.part2(i)) + customer_distances_depot(soln.part2(i) + 1, soln.part1(soln.part4(i)) + 1);
                else
                    flight_time = customer_distances(soln.part1(soln.part3(i)), soln.part2(i)) + customer_distances(soln.part2(i), soln.part1(soln.part4(i)));
                end
            
                if  flight_time > max_flight_time
                    feasible = 0;
                    %fprintf('drone flight times are above the maximum amount\n')
                    %soln
                    break
                end
            end
        end
    end

end

%-----------------------------------------------------------------------%

function[] = plot_stuff(soln, customer_locations)

    figure(1);
    clf;
    sorted_customer_locations = zeros(length(soln.part1), 2);
    sorted_customer_locations(1, :) = 0;
    sorted_customer_locations(length(soln.part1), :) = 0;
    for     icustomer = 2 : length(soln.part1) - 1
            sorted_customer_locations(icustomer,:) = customer_locations(soln.part1(icustomer),:);
    end
    hold on
    plot(0, 0, 'ro')
    plot(customer_locations(:, 1), customer_locations(:, 2), 'ko')

    [rows, ~] = size(customer_locations);
    for icustomer = 1 : rows
        text(customer_locations(icustomer, 1) - 1, customer_locations(icustomer, 2) - 3, sprintf('%d', icustomer))

    end

    plot(sorted_customer_locations(:, 1), sorted_customer_locations(:, 2), 'b-');
    
    
    colors = ['g--'; 'm--'; 'r--'; 'c--'; 'y--'];
    drone_counter = 1;

    for i = 1 : length(soln.part2)
        if  soln.part2(i) ~= -1
            if  soln.part3(i) == 1
                x1 = 0;
                y1 = 0;
            else
                x1 = customer_locations(soln.part1(soln.part3(i)), 1);
                y1 = customer_locations(soln.part1(soln.part3(i)), 2);
            end
            plot([x1, customer_locations(soln.part2(i), 1)], ...
                [y1, customer_locations(soln.part2(i), 2)], colors(drone_counter, :))
            if  soln.part1(soln.part4(i)) == 0
                x2 = 0;
                y2 = 0;
            else
                x2 = customer_locations(soln.part1(soln.part4(i)), 1);
                y2 = customer_locations(soln.part1(soln.part4(i)), 2);
            end
            plot([customer_locations(soln.part2(i), 1), x2], ...
                [customer_locations(soln.part2(i), 2), y2], colors(drone_counter, :))
        else
            drone_counter = drone_counter + 1;
        end

    end
    title('Final Solution')
    hold off

end

%-----------------------------------------------------------------------%

function[] = print_matrix(matrix)

    % size of matrix
    [M, N] = size(matrix);

    %Print
    for     irow = 1 : M
            for     icol = 1 : N
                    fprintf('%9.2f', matrix(irow, icol))
            end
            fprintf('\n')
    end
    fprintf('\n')
end







