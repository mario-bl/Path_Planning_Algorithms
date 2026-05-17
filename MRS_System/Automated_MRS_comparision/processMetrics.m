function [] = processMetrics(APF_Data,ACO_Data,row)

%*** APF Metrics ***%
APF_timeMean = mean(APF_Data.t(1:row));
APF_timeSdv = std(APF_Data.t(1:row));
%** Robot 1 **%
APF_rob1_lMean = mean(APF_Data.rob1.d(1:row));
APF_rob1_lStd = std(APF_Data.rob1.d(1:row));

APF_rob1_wlMean = mean(APF_Data.rob1.wl(1:row));
APF_rob1_wlStd = std(APF_Data.rob1.wl(1:row));
%** Robot 2 **%
APF_rob2_lMean = mean(APF_Data.rob2.d(1:row));
APF_rob2_lStd = std(APF_Data.rob2.d(1:row));

APF_rob2_wlMean = mean(APF_Data.rob2.wl(1:row));
APF_rob2_wlStd = std(APF_Data.rob2.wl(1:row));

fprintf("//*** APF Metrics ***// \n")
fprintf("Average time: %.4f s with std: %.4f s\n",APF_timeMean,APF_timeSdv);
fprintf("Robot 1: \n")
fprintf("Avg distance: %.4f dm with std: %.4f\n",APF_rob1_lMean, APF_rob1_lStd);
fprintf("Avg curvature: %.4f rad/dm with std: %.4f\n",APF_rob1_wlMean, APF_rob1_wlStd);
fprintf("Robot 2: \n");
fprintf("Avg distance: %.4f dm with std: %.4f\n",APF_rob2_lMean, APF_rob2_lStd);
fprintf("Avg curvature: %.4f rad/dm with std: %.4f\n",APF_rob2_wlMean, APF_rob2_wlStd);

%*** ACO METRICS ***%
ACO_timeMean = mean(ACO_Data.t(1:row));
ACO_timeSdv = std(ACO_Data.t(1:row));
%** Robot 1 **%
ACO_rob1_lMean = mean(ACO_Data.rob1.d(1:row));
ACO_rob1_lStd = std(ACO_Data.rob1.d(1:row));

ACO_rob1_wlMean = mean(ACO_Data.rob1.wl(1:row));
ACO_rob1_wlStd = std(ACO_Data.rob1.wl(1:row));
%** Robot 2 **%
ACO_rob2_lMean = mean(ACO_Data.rob2.d(1:row));
ACO_rob2_lStd = std(ACO_Data.rob2.d(1:row));

ACO_rob2_wlMean = mean(ACO_Data.rob2.wl(1:row));
ACO_rob2_wlStd = std(ACO_Data.rob2.wl(1:row));
fprintf("//*** ACO Metrics ***// \n")
fprintf("Average time: %.4f s with std: %.4f s\n",ACO_timeMean,ACO_timeSdv);
fprintf("Robot 1: \n")
fprintf("Avg distance: %.4f dm with std: %.4f\n",ACO_rob1_lMean, ACO_rob1_lStd);
fprintf("Avg curvature: %.4f rad/dm with std: %.4f\n",ACO_rob1_wlMean, ACO_rob1_wlStd);
fprintf("Robot 2: \n");
fprintf("Avg distance: %.4f dm with std: %.4f\n",ACO_rob2_lMean, ACO_rob2_lStd);
fprintf("Avg curvature: %.4f rad/dm with std: %.4f\n",ACO_rob2_wlMean, ACO_rob2_wlStd);


end