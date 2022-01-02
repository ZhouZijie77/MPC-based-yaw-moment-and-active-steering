function [Tbfl,Tbfr,Tbrl,Tbrr] = braking_cal(M,alphaf,alphar,r,c)
%Input:M,alphaf,alphar
%Output:Tbfl,Tbfr,Tbrl,Tbrr
%a,b,c¶¼Îª³µÁ¾³ß´ç

Tbfl = -M*r/c;
Tbfr = M*r/c;
Tbrl = -M*r/c;
Tbrr = M*r/c;
% if M==0
%     Tbfl=0;
%     Tbfr=0;
%     Tbrl=0;
%     Tbrr=0;
% elseif M>0
%     if alphaf>alphar
%         Tbfl=0;
%         Tbfr=0;
%         Tbrl=abs(M)*r/c;
%         Tbrr=0;
%     else
%         Tbfl=abs(M)*r/c;
%         Tbfr=0;
%         Tbrl=0;
%         Tbrr=0;
%     end
% else
%     if alphaf>alphar
%         Tbfl=0;
%         Tbfr=0;
%         Tbrl=0;
%         Tbrr=abs(M)*r/c;
%     else
%         Tbfl=0;
%         Tbfr=abs(M)*r/c;
%         Tbrl=0;
%         Tbrr=0;
%     end     
% end
end

