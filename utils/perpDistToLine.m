function d = perpDistToLine(linept1, linept2, point)
    v01 = point - linept1;
    v02 = point - linept2;
    v21 = linept2 - linept1;
    
    d = norm(cross(v01,v02)) / norm(v21);
end