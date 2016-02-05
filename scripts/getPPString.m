function output = getPPString(time, waypoints)

    piecewise_polyn = pchip(time, waypoints);
    output = '';
    
    for piece = 1:piecewise_polyn.pieces
       duration = piecewise_polyn.breaks(piece + 1) - piecewise_polyn.breaks(piece)
       output = sprintf('%s%.5f\n%i \n', output, duration, piecewise_polyn.dim);
       for dim = 1:piecewise_polyn.dim
            chunk = piecewise_polyn.coefs(piece*piecewise_polyn.dim+dim-piecewise_polyn.dim,:);
            output = sprintf('%s%f %f %f %f \n', output, chunk(4), chunk(3), chunk(2), chunk(1));
       end
    end
end