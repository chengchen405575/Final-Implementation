function U = embedded_babai_candidate(input)
%EMBEDDED_BABAI_CANDIDATE Forward nearest-plane point for embedded H.
H=double(input.H); Uunc=double(input.U_unc(:)); z=H*Uunc; n=numel(Uunc);
U=zeros(n,1);
for i=1:n
    head=0; if i>1, head=H(i,1:i-1)*U(1:i-1); end
    U(i)=double((z(i)-head)/H(i,i)>0.5);
end
end
