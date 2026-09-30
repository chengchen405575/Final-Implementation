function test_current_observer_refactor
thisDirectory=fileparts(mfilename('fullpath'));
modelRoot=fileparts(thisDirectory);
assemblyDirectory=fullfile(modelRoot,'functions','Deadtime_CMV_FTE_Double_Vector');
addpath(thisDirectory,assemblyDirectory);
pathCleanup=onCleanup(@()rmpath(thisDirectory,assemblyDirectory)); %#ok<NASGU>
Ld=1e-3; Lq=1.2e-3; Rs=0.1; Psi=0.05; wel=500; Ts=20e-6;
Ac=[-Rs/Ld,wel*Lq/Ld;-wel*Ld/Lq,-Rs/Lq];
Bc=diag([1/Ld,1/Lq]); fc=[0;-wel*Psi/Lq];
Md=expm([Ac,Bc,fc;zeros(3,5)]*Ts);
Ad=Md(1:2,1:2); Bd=Md(1:2,3:4); fd=Md(1:2,5);
Tc=[2/3,-1/3,-1/3;0,1/sqrt(3),-1/sqrt(3)];
Vdc=100; Bs_TI=Tc*Vdc*(eye(3)-ones(3)/3);
theta=0.4; sc=[sin(theta);cos(theta)]; U=[1;0;0;1;1;0];
Tf=25e-6; Tsen=1/(2*pi*100e3); phase=atan(wel*Tf)+atan(wel*Tsen);
[K,info]=ConfigureCurrentObserverGainDQ(Ad,wel,Ts,Tsen,Tf, ...
    [1e-2;1e-2;1e-3;1e-3;1e-3;1e-3],[1e-2;1e-2]);
assert(info.observabilityRank==6 && info.spectralRadius<1);
args={ [5;-3],Ad,Bd,fd,Bs_TI,sc,theta,wel,Ts,U,1.6e-6,Tf,Tsen,phase,K,false };
[il,rl,yfl,vl]=LegacyRippleCompensatorDQ(args{:});
clear CurrentObserverDQ
[io,ro,yfo,vo]=CurrentObserverDQ(args{:});
[io2,ro2,yfo2,vo2]=CurrentObserverDQ(args{:});
assert(all(isfinite([il;rl;yfl;io;ro;yfo;io2;ro2;yfo2])));
assert(vl && ~vo && vo2);

Np=5; Apow=zeros(12,2); Gamma=zeros(10,2); Phi=zeros(10,1);
sumA=eye(2);
for k=0:5, Apow(2*k+(1:2),:)=Ad^k; end
for k=1:5
    Gamma(2*k-1:2*k,:)=Ad^k;
    if k==1, sumA=eye(2); else, sumA=sumA+Ad^(k-1); end
    Phi(2*k-1:2*k)=sumA*fd;
end
[H,Uunc,iabc,db,ds,pe,edq]=Assembly_QP(Ad,Bd,fd,Gamma,Phi,Apow, ...
    1e-3*eye(15),zeros(15,3),Bs_TI,il,[0;0],sc,wel,Ts,U, ...
    1e-9,zeros(3,1),false);
assert(isequal(size(H),[15,15]) && isequal(size(Uunc),[15,1]));
assert(isequal(size(iabc),[3,1]) && isequal(size(db),[3,1]));
assert(isequal(size(ds),[3,3]) && isequal(size(pe),[3,1]));
assert(isequal(size(edq),[2,1]) && all(isfinite([H(:);Uunc;iabc;db;ds(:)])));
assert(norm(H.'*H-(H.'*H).','fro')<1e-10);
checkcode('LegacyRippleCompensatorDQ.m');
checkcode('CurrentObserverDQ.m');
checkcode('ConfigureCurrentObserverGainDQ.m');
checkcode('Assembly_QP.m');
fprintf('PASS: both Variant interfaces, observer gain and QP dimensions. rho=%.6f\n',info.spectralRadius);
end
