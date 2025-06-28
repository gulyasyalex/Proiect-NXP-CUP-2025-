function [X, Y] = plotLineABC(line, xmin, xmax, ymin, ymax)
%PLOTLINEABC Summary of this function goes here
%   Detailed explanation goes here
    a = line(1);
    b = line(2);
    c = line(3);
    if b ~= 0
        a = a/b;
        c = c/b;
        b = 1;
        x = [xmin xmax];
        y = ((-a).*x) -c;
        X = x;
        Y = y;
        % plot(x,y)
    else
        c = c / a;
        x = -c;
        y = [ymin ymax];
        X = [x x];
        Y = y;
        %plot([x, x],y)
    end
end

