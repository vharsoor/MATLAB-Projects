%% Calling Simulink model and security

clc
clear all

Gain_LCW = 90000;
Gain_HCW = 500000;
InitSpeed = 40; 
decelLim = -150;
%decelLim = -200;
switches = 0;
%reset(RandStream.getGlobalStream,sum(100*clock));

[A, B, C, D, Kess, Kr, Ke, uD] = designControl(secureRand(), Gain_HCW);


for InitSpeed = 20:40 
    open_system('LaneMaintainSystem.slx')
    
    set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
    set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(InitSpeed))
    
    simModel1 = sim('LaneMaintainSystem.slx');
    % Access the simulation output data
    simOut1 = simModel1.get('sx1');
    
    % Extract the time and data vectors
    stime = simOut1.time;
    sdist = simOut1.data;
    
    % Display the time and data
    disp('Stopping Time:');
    disp(stime(end));
    disp('Stopping Distance:');
    disp(sdist(end));

    for i = 1:length(sdist)
        if sdist(i) >= 0
            tc=stime(i);
            break
        else
            tc = 0;
        end
    end

    fprintf("collision time = %.4f\n", tc);
    
    if sdist(end) >= 0
        % Input Data for HR and RR
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
                HR_LCW_samples(sample_idx, i) = max(normrnd(HR_LCW_means(i), HR_LCW_stddevs(i)),1);
                HR_HCW_samples(sample_idx, i) = max(normrnd(HR_HCW_means(i), HR_HCW_stddevs(i)),1);
                RR_LCW_samples(sample_idx, i) = max(normrnd(RR_LCW_means(i), RR_LCW_stddevs(i)),1);
                RR_HCW_samples(sample_idx, i) = max(normrnd(RR_HCW_means(i), RR_HCW_stddevs(i)),1);
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
        t0_LCW = 0.01 * LCW_Rq;
        t0_HCW = 0.01 * HCW_Rq;

        
        % Display the values of Rq and t0 for LCW and HCW
        fprintf('Respiratory Quotient (Rq) for LCW: %.4f\n', LCW_Rq);
        fprintf('t0 for LCW: %.6f\n', t0_LCW);
        
        fprintf('Respiratory Quotient (Rq) for HCW: %.4f\n', HCW_Rq);
        fprintf('t0 for HCW: %.6f\n', t0_HCW);

    
        % Load the HumanActionModel.slx model
        model = 'HumanActionModel';
        open_system(model);
    
        % Define the values for step and Final_Value
        step_value = t0_HCW; % Set this to the desired value of t0_HCW
        final_value = 1.1 * decelLim;  % Set this to the desired final value
    
        % Set the step and Final_Value parameters of the Step block
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
    
        %Check if hstop<collision time, if yes, switch to human, else, don't switch
        if hstop < tc
            amax = 1.1 * decelLim;
            fprintf('amax: %.4f\n', amax);
            disp('Switched to Human');
            switches = switches+1;
        else
            amax = decelLim;
            fprintf('amax: %.4f\n', amax);
            disp("Colliding, switching to huamn can't save us")
        end
    end
end

fprintf('No. of Switches to Human = %i\n', switches);

 