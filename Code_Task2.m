%% Test code in class

clc
clear all

close all

%% Conditions for rainy road
%Changing gain and trying to minimizing switches
Gain_LCW = 90000;
Gain_HCW = 500000;
%Gain = 500000;
%InitSpeed = 50; 
%decelLim = -150;

NCollisions = 0;

NSwitches = 0;
f=0.002;

% Reset the random number generator's seed for randomness
rng('shuffle');

numSteps = 100;
Collision_Array = [];
Switch_Array = [];

P = [0.6 0.4; 0.85 0.15];

mc = dtmc(P);

scenario = simulate(mc,numSteps); % 2 is HCW, 1 is LCW
%disp(scenario);

%Define the HR & RR mean and std deviation fro User 3
HR_LCW_3 = [61, 14];
RR_LCW_3 = [17, 8];
HR_HCW_3 = [92, 23];
RR_HCW_3 = [26, 16];

% Define the range
Speed = [20, 60];

% Define mean and standard deviation of the Gaussian distribution
mu = (Speed(2) + Speed(1)) / 2;  % Mean
sigma = (Speed(2) - Speed(1)) / 4;  % Standard Deviation

%define the Number of steps/scenarios
%numSteps = 100;
HR_LCW_3_samples = zeros(numSteps, 1);
RR_LCW_3_samples = zeros(numSteps, 1);
Speed_samples = zeros(numSteps, 1);

rng('shuffle');

for sample_idx = 1:numSteps
    % Sample for HR/RR for LCW/HCW
    HR_LCW_3_samples(sample_idx, 1) = max(normrnd(HR_LCW_3(1), HR_LCW_3(2)),1);
    RR_LCW_3_samples(sample_idx, 1) = max(normrnd(RR_LCW_3(1), RR_LCW_3(2)),1);
    HR_HCW_3_samples(sample_idx, 1) = max(normrnd(HR_HCW_3(1), HR_HCW_3(2)),1);
    RR_HCW_3_samples(sample_idx, 1) = max(normrnd(RR_HCW_3(1), RR_HCW_3(2)),1);

    %Calculate Rq_LCW and Rq_HCW for these random samples
    

    % Sample from a truncated normal distribution within the specified range
    sample = normrnd(mu, sigma);
    
    % Check if the sample is within the specified range, and adjust if necessary
    if sample < Speed(1)
        Speed_samples(sample_idx, 1) = Speed(1);
    elseif sample > Speed(2)
        Speed_samples(sample_idx, 1) = Speed(2);
    else
        Speed_samples(sample_idx, 1) = sample;
    end
end


% Loop through the iterations
for iteration = 1:numSteps
    % Pick a value from the 'scenario' array
    scenarioValue = scenario(iteration);
    
    % Decide decelLim based on the 'scenarioValue'
    if scenarioValue == 1
        decelLim = -200;
        [A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain_LCW);
    elseif scenarioValue == 2
        decelLim = -150;
        [A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain_HCW);
    end
    
    % Pick the corresponding 'initSpeed' value from the 'Speed_samples' array
    InitSpeed = Speed_samples(iteration);
    %InitSpeed = 33;

    open_system('LaneMaintainSystem.slx')
    
    set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
    set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(InitSpeed))
    
    simModel1 = sim('LaneMaintainSystem.slx');
    % Access the simulation output data
    simOut1 = simModel1.get('sx1');
    
    % Extract the time and data vectors
    stime = simOut1.time;
    sdist = simOut1.data;
    disp('Distance:');
    disp(sdist(end));

    %fprintf("collision time = %.4f\n", tc);

    if sdist(end) < 0
        Collision_Array = [Collision_Array, 'no '];
        Switch_Array = [Switch_Array, 'no '];
        disp("No switch, No Collision - Autonomus takes care");
    else
        %Check if the Average user Model based Advisory control predicts a
        %switch or not
        HR_LCW_means = [80, 65, 61];
        HR_HCW_means = [95, 71, 92];
        HR_LCW_stddevs = [14, 15, 14];
        HR_HCW_stddevs = [26, 21, 23];
        
        RR_LCW_means = [16, 13, 17];
        RR_HCW_means = [21, 14, 26];
        RR_LCW_stddevs = [6, 4, 8];
        RR_HCW_stddevs = [14, 5, 16];
        
        % Number of samples
        num_samples = 100; % You can adjust this number as needed
        
        % Initialize variables to store samples
        HR_LCW_samples = zeros(num_samples, length(HR_LCW_means));
        HR_HCW_samples = zeros(num_samples, length(HR_HCW_means));
        RR_LCW_samples = zeros(num_samples, length(RR_LCW_means));
        RR_HCW_samples = zeros(num_samples, length(RR_HCW_means));
        
        % Reset the random number generator's seed for randomness
        rng('shuffle');
        
        % Generate random samples for HR and RR for LCW and HCW
        for sample_idx = 1:num_samples
            for i = 1:length(HR_LCW_means)
                % Randomly select values of HR and RR based on the Gaussian distribution
                HR_LCW_samples(sample_idx, i) = normrnd(HR_LCW_means(i), HR_LCW_stddevs(i));
                HR_HCW_samples(sample_idx, i) = normrnd(HR_HCW_means(i), HR_HCW_stddevs(i));
                RR_LCW_samples(sample_idx, i) = normrnd(RR_LCW_means(i), RR_LCW_stddevs(i));
                RR_HCW_samples(sample_idx, i) = normrnd(RR_HCW_means(i), RR_HCW_stddevs(i));
            end
        end
        
        % Calculate the averages for LCW and HCW
        HR_LCW_avg = mean(HR_LCW_samples, 1);
        HR_HCW_avg = mean(HR_HCW_samples, 1);
        RR_LCW_avg = mean(RR_LCW_samples, 1);
        RR_HCW_avg = mean(RR_HCW_samples, 1);
        
        % Calculate Rq for LCW and HCW
        LCW_Rq = mean(HR_LCW_avg) / mean(RR_LCW_avg);
        HCW_Rq = mean(HR_HCW_avg) / mean(RR_HCW_avg);
                
        % Calculate t0 for LCW and HCW
        t0_LCW = f * LCW_Rq;
        t0_HCW = f * HCW_Rq;

        if scenarioValue == 1
            tr = t0_LCW;
            Gain = Gain_LCW;
        else
            tr = t0_HCW;
            Gain = Gain_HCW;
        end

        %Run the advisory control(using average user model) to see if that
        %switches or not
        [Switch,Collision] = AdvisoryControl(InitSpeed, decelLim, tr, Gain);

        %If it switches, let's check if User 3 can stop the car
        if Switch
            %Increases switches
            
            %Sample tr from User 3
            %But first check again if its LCW or HCW Condition
            if scenarioValue == 1   %User with LCW
                Hr = HR_LCW_3_samples(iteration);
                Rr = RR_LCW_3_samples(iteration);
                tr = f * Hr/Rr;
                Gain = Gain_LCW;
                [Switch,Collision] = AdvisoryControl(InitSpeed, decelLim, tr, Gain);
                if Collision
                    %User 3 collides in this scenario, increase collisions
                    NCollisions = NCollisions + 1;
                    Collision_Array = [Collision_Array, 'yes '];
                    Switch_Array = [Switch_Array, 'no '];
                else
                    %User 3 can handle it, switch to him
                    NSwitches = NSwitches + 1;
                    Collision_Array = [Collision_Array, 'no '];
                    Switch_Array = [Switch_Array, 'yes '];
                end
            else  
                %Check Collision for HCW
                Hr = HR_HCW_3_samples(iteration);
                Rr = RR_HCW_3_samples(iteration);
                tr = f * Hr/Rr;
                Gain = Gain_HCW;
                [Switch,Collision] = AdvisoryControl(InitSpeed, decelLim, tr, Gain);
                if Collision
                    %User 3 collides in this scenario, increase collisions
                    NCollisions = NCollisions + 1;
                    Collision_Array = [Collision_Array, 'yes '];
                    Switch_Array = [Switch_Array, 'no '];
                else
                    %User 3 can handle it, switch to him
                    NSwitches = NSwitches + 1;
                    Collision_Array = [Collision_Array, 'no '];
                    Switch_Array = [Switch_Array, 'yes '];
                end
            end
        else
            %Even human can't stop collision, so don't switch, increase
            %Collisions
            NCollisions = NCollisions + 1;
            Collision_Array = [Collision_Array, 'yes '];
            Switch_Array = [Switch_Array, 'no '];
        end
    end
end

fprintf("No. Of Switches=%i\nNo. of Collisions=%i\n", NSwitches, NCollisions);


%Advisory Control Function Definition
function [Switch, Collision] = AdvisoryControl(InitSpeed, decelLim, tr, Gain)
    %Run the autonous braking system first, to check if it can stop the car
    %own its own

    [A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);

    %open_system('Copy_2_of_LaneMaintainSystem.slx')
    open_system('LaneMaintainSystem.slx')
    
    %set_param('Copy_2_of_LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
    %set_param('Copy_2_of_LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(InitSpeed))
    set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
    set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(InitSpeed))
    
    %simModel1 = sim('Copy_2_of_LaneMaintainSystem.slx');
    simModel1 = sim('LaneMaintainSystem.slx');

    % Access the simulation output data
    simOut1 = simModel1.get('sx1');
    
    % Extract the time and data vectors
    stime = simOut1.time;
    sdist = simOut1.data;
    
    % Display the time and data
    %disp('Stopping Time:');
    %disp(stime(end));
    %disp('Stopping Distance:');
    %disp(sdist(end));

    for i = 1:length(sdist)
        if sdist(i) >= 0
            tc=stime(i);
            break
        else
            tc = 0;
        end
    end

    fprintf("collision time = %.4f\n", tc);
    
    %if the autonous braking system couldn't stop the car, let's see if a
    %average human can
    if sdist(end) < 0
        Switch = 0;
        Collision = 0;
        return
    else
        % Load the LaneMaintainSystem0x2DCopy.slx model
        model = 'HumanActionModel';
        open_system(model);
    
        % Define the values for step and Final_Value
        step_value = tr; % Set this to the desired value of t0_HCW
        final_value = 1.1 * decelLim;  % Set this to the desired final value
    
        % Set the step and Final_Value parameters of the Step block
        fprintf("tr=%.4f\n",tr);
        step_block_path = [model '/Step']; % Specify the block name as 'step'
        set_param(step_block_path, 'time', num2str(step_value));
        set_param(step_block_path, 'after', num2str(final_value));
        set_param('HumanActionModel/VehicleKinematics/vx','InitialCondition', num2str(InitSpeed))
    
        % Simulate the model
        simModel = sim(model);
        simOut = simModel.get('sx1');
    
        % Extract the total time (hstop) attribute from the model
        hstop = max(simOut.time); % Extract the maximum simulation time
        hdist = max(simOut.data);
    
        % Display the total time (hstop)
        fprintf('Total Time (hstop): %.4f\n', hstop);
        fprintf('Stopping Distance(hdist): %.4f\n', hdist);
    
        % Close the model
        close_system(model, 0);
    
        %Check if hdist < 0, if yes, switch to human, else, don't switch
        if hstop < tc
            amax = 1.1 * decelLim;
            fprintf('amax: %.4f\n', amax);
            disp('Switched to Human');
            Switch = 1;
            Collision = 0;
            return
        else
            amax = decelLim;
            fprintf('amax: %.4f\n', amax);
            disp("Colliding, switching to huamn can't save us")
            Switch = 1;
            Collision = 1;
            return
        end
    end
end
