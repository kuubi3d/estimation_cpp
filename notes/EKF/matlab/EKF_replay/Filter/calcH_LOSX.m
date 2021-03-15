function H_LOSX = calcH_LOSX(q0,q1,q2,q3,range,vd,ve,vn)
%CALCH_LOSX
%    H_LOSX = CALCH_LOSX(Q0,Q1,Q2,Q3,RANGE,VD,VE,VN)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    29-May-2017 00:16:14

t2 = 1.0./range;
H_LOSX = [t2.*(q1.*vd.*2.0+q0.*ve.*2.0-q3.*vn.*2.0),t2.*(q0.*vd.*2.0-q1.*ve.*2.0+q2.*vn.*2.0),t2.*(q3.*vd.*2.0+q2.*ve.*2.0+q1.*vn.*2.0),-t2.*(q2.*vd.*-2.0+q3.*ve.*2.0+q0.*vn.*2.0),-t2.*(q0.*q3.*2.0-q1.*q2.*2.0),t2.*(q0.^2-q1.^2+q2.^2-q3.^2),t2.*(q0.*q1.*2.0+q2.*q3.*2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
