function flag = myinpolygon(X,Y,x0,y0,x1,y1,x2,y2,x3,y3,x4,y4)
% decide if the points are inside or outside the polygon
% Input:
%   X: x-coordinations of the points to be detected
%   Y: y-coordinations of the points to be detected
%   xv: x-coordinations of the vertex define the polygon
%   yv: y-coordinations of the vertex define the polygon
% Output:
%   flag=1: inside the polygon
%   flag=0: outside the polygon
% Poweed by Xianbao Duan
% Email: xianbao.d@gmail.com
% Website: http://www.52souji.net/
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2012-05-07: Complete
% 2012-05-18: Modify the comments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
vertex(:,1) = xv;
vertex(:,2) = yv;
% generate the Delaunay Triangle
dt = DelaunayTri(vertex);
% calculate the volumn of the body
k = convexHull(dt);
 
xv2 = dt.X(k(1:(end-1)),1);
yv2 = dt.X(k(1:(end-1)),2);
 
flag = inpolygon(X,Y,xv2,yv2);