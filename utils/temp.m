x = linspace(0,0.3,10^6);

tic
for i=1:length(x)
    acosd(x(i));
end
toc


tic
for i=1:length(x)
    acosdFast(x(i));
end
toc