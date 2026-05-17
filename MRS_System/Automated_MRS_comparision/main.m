%ARCHVIO DE EJECUCION PARA SIMILAR VARIEDAD DE SITUACIONES 
%EL OBJETIVO ES PROBAR LOS DOS ALGORITMOS EN DISTINTAS DISTRIBUCIONES DE
%OBJSTACULOS PARA VER QUE TAN BIEN REACCIONAN Y SACAR LAS MÉTRICAS
%CORRESPONDIETES PARA LA POSTERIOR COMPARACIÓN

%Semilla 40 y 42 bastante guay
% addpath("IACO_MRS\")
% runAlgorithmIACO

clear; clc;
close all;

t0 = tic;

trajectories(1) = struct('x0', 0,  'y0', 0,  'xObj', 18, 'yObj', 18);
trajectories(2) = struct('x0', 0,  'y0', 18, 'xObj', 18, 'yObj', 0);
n_filas = 20;
n_cols = n_filas;
nObs = 10;
diffStages = 20;
runs = 5;
maxData = diffStages * runs;
rng('shuffle');
seeds = randi([0, 80], 1, diffStages);

addpath("IAPF_MRS\")
addpath("IACO_MRS\")

Robot1Data = struct('d',zeros(maxData,1),'wl',zeros(maxData,1));
Robot2Data = Robot1Data;

APF_Data = struct('rob1',Robot1Data,'rob2',Robot2Data,'t',zeros(maxData,1));
ACO_Data = struct('rob1',Robot1Data,'rob2',Robot2Data,'t',zeros(maxData,1));

 
succesfullAPF = maxData;
succesfullACO = maxData;
apfColumn = 1;
acoColumn = 2;


indexMatrix = 0:runs:diffStages*runs;
plotSimulations = false;

row = 0;

for i = 1:length(seeds)
        
    obs = generateScenario(n_filas, n_cols, nObs , trajectories, seeds(i));  % semilla 42
    mapa = buildObstacleMapV2(n_filas, n_cols, obs);  

    for j = 1:runs
        if (i-1)*runs + j == maxData
            plotSimulations = true;
        end
        %**** ALGORITHMs deployed ****%
        [metricsAPF, timeAPF, failedAlgthAPF]= runAlgorithmAPF(obs,trajectories,plotSimulations);
        [metricsACO, timeACO, failedAlgthACO]= runAlgorithmIACO(mapa,trajectories,plotSimulations);
        
        if plotSimulations
            totalTime = toc(t0);
        end

        if failedAlgthACO
            succesfullACO = succesfullACO - 1;
        end

        if failedAlgthAPF
            succesfullAPF = succesfullAPF - 1; 
        end

        if ~failedAlgthAPF && ~failedAlgthACO
            fprintf("New run succesfull \n");
            row = row + 1;
            %*** APF Data update ***%
            APF_Data.rob1.d(row) = metricsAPF(1).d;
            APF_Data.rob1.wl(row) = metricsAPF(1).wl;

            APF_Data.rob2.d(row) = metricsAPF(2).d;
            APF_Data.rob2.wl(row) = metricsAPF(2).wl;
            
            APF_Data.t(row) = timeAPF;
            
            %*** ACO Data update ***%
            ACO_Data.rob1.d(row) = metricsACO(1).d;
            ACO_Data.rob1.wl(row) = metricsACO(1).wl;

            ACO_Data.rob2.d(row) = metricsACO(2).d;
            ACO_Data.rob2.wl(row) = metricsACO(2).wl;
            
            ACO_Data.t(row) = timeACO;
            
        end
    end
end


fprintf("APF Succesfull runs: %.2f\n", 100 *succesfullAPF/maxData );
fprintf("ACO Succesfull runs: %.2f\n", 100 *succesfullACO/maxData );
processMetrics(APF_Data, ACO_Data, row);
fprintf("Total time spended: %.2f s\n",totalTime);



function [] = plotAPFobstaculos(obstacles, n_filas, n_col)

    figure;
    hold on;
    xlim([-1,n_col + 1]);
    ylim([-1,n_filas + 1]);
    % axis equal;
    grid on;

    for i = 1:numel(obstacles)
        xo = obstacles(i).x;
        yo = obstacles(i).y;
        ro = obstacles(i).r;
        co = obstacles(i).colour;
        rectangle('Position',[xo-ro, yo-ro, 2*ro, 2*ro],...
                  'Curvature',[1 1],...
                  'FaceColor',co,...
                  'EdgeColor','k');
    end
end

function [] = plotACOobstacules(mapa, n_filas,n_cols)
figure
imagesc(-1:n_filas +1, -1:n_cols + 1, mapa)
hold on
set(gca, 'YDir', 'normal')

end