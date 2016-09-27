function [Rout,tout] = UrbanRel(Xl, Xr)

nrPts = size(Xl,2);
r = zeros(3,nrPts);
s = zeros(3,nrPts);

for i = 1:nrPts
    null_r = null(Xr(1:3,i)');
    r(:,i) = null_r(:,1);
    s(:,i) = null_r(:,2);
end

A = zeros(2*nrPts,12);
for i=1:nrPts
   A (2*i-1,1) = r(1,i)*Xl(1,i);
   A (2*i,1)   = s(1,i)*Xl(1,i);
       
   A (2*i-1,2) = r(1,i)*Xl(2,i);
   A (2*i,2)   = s(1,i)*Xl(2,i);

   A (2*i-1,3) = r(1,i)*Xl(3,i);
   A (2*i,3)   = s(1,i)*Xl(3,i);
   
   A (2*i-1,4) = r(2,i)*Xl(1,i);
   A (2*i,4)   = s(2,i)*Xl(1,i);
   
   A (2*i-1,5) = r(2,i)*Xl(2,i);
   A (2*i,5)   = s(2,i)*Xl(2,i); 
   
   A (2*i-1,6) = r(2,i)*Xl(3,i);    
   A (2*i,6)   = s(2,i)*Xl(3,i);

   A (2*i-1,7) = r(3,i)*Xl(1,i);
   A (2*i,7)   = s(3,i)*Xl(1,i);
   
   A (2*i-1,8) = r(3,i)*Xl(2,i);
   A (2*i,8)   = s(3,i)*Xl(2,i);
   
   A (2*i-1,9) = r(3,i)*Xl(3,i);
   A (2*i,9)   = s(3,i)*Xl(3,i);

   A (2*i-1,10) = r(1,i);
   A (2*i,10)   = s(1,i);
   
   A (2*i-1,11) = r(2,i);
   A (2*i,11)   = s(2,i);
 
   A (2*i-1,12) = r(3,i);
   A (2*i,12)   = s(3,i);
end

[~,~,res] = svd(A'*A);

P = reshape(res(1:9,end),3,3);
% scalefact = (abs(norm(P(:,1))*norm(P(:,2))*norm(P(:,3))))^(1/3);

%  tout = R*(t./scalefact);
[U2,~,V2] = svd(P(1:3,1:3)); %SVD to find the best rotation matrix in the Frobenius sense
% d=sign(det(U2*V2'));
Rout = U2*V2';
if det(Rout) < 0
     Rout = -1*Rout;
end


% t1 = abs(cross(cross(Xl(:,1),R*Xr(:,1)),cross(Xl(:,2),R*Xr(:,2))));
% for i=1:5
%    cross(cross(Xl(:,i),Rout*Xr(:,i)),cross(Xl(:,i+1),Rout*Xr(:,i+1))) 
% end
f2prime1 = Rout*Xr(:,1);
f2prime2 = Rout*Xr(:,2);

normal1 = cross(Xl(:,1),f2prime1);
normal2 = cross(Xl(:,2),f2prime2);

tout = cross(normal1,normal2);
tout = tout ./ norm(tout);
opticalFlow = Xl(:,1) - f2prime1;
if (dot(opticalFlow,tout) < 0)
    tout = -tout;
end                   


% rmin = rot2cayley(Rout)';
% % test = t1./norm(t1);
% 
% x0 = [rmin];
% lb = [-inf*ones(1,3)];
% up = [inf*ones(1,3)];
% options = optimset('Display','iter','Jacobian','on','DerivativeCheck','off');
% 
% x = lsqnonlin(@errorNull,x0,lb,up,options,[Xl;Xr]);
% 
% Rout = cayley2rot(x);
% tout = x(4:6)';
end