%
%
%

clear all;
close all;

load usrp_rx_gain.dat

% signal bandwidth
w = usrp_rx_gain(:,1);  w = w(:)';

% noise level
%n = usrp_rx_gain(:,2); n = n(:)';

% signal level
s = usrp_rx_gain(:,3); s = s(:)';

decim = 16e6 ./ w;

% compute gain correction factor
N = length(w);
s_hat = zeros(1,N);
gain = zeros(1,N);


t1 = log2([10 18 34 66 130]');
m1 = [0.42 -1.28 -2.37 -4.23 -5.91]';
pm1 = polyfit(t1,m1,1);

t2 = log2([16 32 64 128]');
m2 = [
    (8.5 + 1.5)
    (8.3 + 2.4)
    (7.8 + 4.2)
    (6.7 + 6.0)];
pm2 = polyfit(t2,m2,1);

for i=1:N,
    d = log2(decim(i)-2);
    f = rem(d,1);
    s_hat(i) = f*polyval(pm2,d) + polyval(pm1,d);
    gain(i) = 10^(-s_hat(i)/10);
end;

semilogx(decim,s,'-x',decim,s_hat,'-x',decim,s-s_hat,'-x');
xlabel('decim rate');
ylabel('signal level');
legend('measured signal level','estimated signal level','corrected',0);
grid on;

