function [public_vars] = uncertainty(read_only_vars,public_vars)

if(read_only_vars.counter < public_vars.measure.n + 1)
public_vars.measure.GNSS.data(read_only_vars.counter,:) = read_only_vars.gnss_position;
public_vars.measure.LIDAR.data(read_only_vars.counter,:) = read_only_vars.lidar_distances;
elseif(read_only_vars.counter == public_vars.measure.n + 1)

public_vars.measure.GNSS.mean = mean((public_vars.measure.GNSS.data));
public_vars.measure.LIDAR.mean = mean((public_vars.measure.LIDAR.data));
public_vars.measure.GNSS.std = std((public_vars.measure.GNSS.data));
public_vars.measure.LIDAR.std = std((public_vars.measure.LIDAR.data));
N=9;

figure(2)
yyaxis left
subplot(3,4,2);
histogram(public_vars.measure.GNSS.data(:,1),N);
hold on
yyaxis right
x=min(public_vars.measure.GNSS.data(:,1)):0.1:max(public_vars.measure.GNSS.data(:,1));
plot(x,norm_pdf(x,public_vars.measure.GNSS.mean(1),public_vars.measure.GNSS.std(1)))
hold off
yyaxis left
subplot(3,4,3);
histogram(public_vars.measure.GNSS.data(:,2),N);
hold on
yyaxis right
x=min(public_vars.measure.GNSS.data(:,2)):0.1:max(public_vars.measure.GNSS.data(:,2));
plot(x,norm_pdf(x,public_vars.measure.GNSS.mean(2),public_vars.measure.GNSS.std(2)))
hold off
yyaxis left
subplot(3,4,5);
histogram(public_vars.measure.LIDAR.data(:,1),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,1)):0.01:max(public_vars.measure.LIDAR.data(:,1));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(1),public_vars.measure.LIDAR.std(1)))
hold off
yyaxis left
subplot(3,4,6);
histogram(public_vars.measure.LIDAR.data(:,2),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,2)):0.01:max(public_vars.measure.LIDAR.data(:,2));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(2),public_vars.measure.LIDAR.std(2)))
hold off
yyaxis left
subplot(3,4,7);
histogram(public_vars.measure.LIDAR.data(:,3),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,3)):0.01:max(public_vars.measure.LIDAR.data(:,3));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(3),public_vars.measure.LIDAR.std(3)))
hold off
yyaxis left
subplot(3,4,8);
histogram(public_vars.measure.LIDAR.data(:,4),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,4)):0.01:max(public_vars.measure.LIDAR.data(:,4));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(4),public_vars.measure.LIDAR.std(4)))
hold off
yyaxis left
subplot(3,4,9);
histogram(public_vars.measure.LIDAR.data(:,5),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,5)):0.01:max(public_vars.measure.LIDAR.data(:,5));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(5),public_vars.measure.LIDAR.std(5)))
hold off
yyaxis left
subplot(3,4,10);
histogram(public_vars.measure.LIDAR.data(:,6),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,6)):0.01:max(public_vars.measure.LIDAR.data(:,6));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(6),public_vars.measure.LIDAR.std(6)))
hold off
yyaxis left
subplot(3,4,11);
histogram(public_vars.measure.LIDAR.data(:,7),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,7)):0.01:max(public_vars.measure.LIDAR.data(:,7));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(7),public_vars.measure.LIDAR.std(7)))
hold off
yyaxis left
subplot(3,4,12);
histogram(public_vars.measure.LIDAR.data(:,8),N);
hold on
yyaxis right
x=min(public_vars.measure.LIDAR.data(:,8)):0.01:max(public_vars.measure.LIDAR.data(:,8));
plot(x,norm_pdf(x,public_vars.measure.LIDAR.mean(8),public_vars.measure.LIDAR.std(8)))
hold off
yyaxis left

public_vars.measure.GNSS.cov = cov(public_vars.measure.GNSS.data);
public_vars.measure.LIDAR.cov = cov(public_vars.measure.LIDAR.data);

figure(3)
x=-1:0.01:1;
plot(x,norm_pdf(x,0,public_vars.measure.GNSS.std(1)),'linewidth',2)
hold on
plot(x,norm_pdf(x,0,public_vars.measure.LIDAR.std(1)),'linewidth',2)
hold off
legend('GNSS','LIDAR')

public_vars.measure.GNSS.data
end