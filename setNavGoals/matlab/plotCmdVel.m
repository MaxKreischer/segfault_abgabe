angVelz = zeros([length(cmdvel00),1]);
for i = 1 : length(cmdvel00)
   if( mod(i,8) == 0 )
       angVelz(i) = cmdvel00(i);
       if(isnan(cmdvel00(i)))
           angVelz(i) = 0;
       end
   end
end
xAxis = (linspace(0,length(angVelz),length(angVelz)))';
plot(xAxis,angVelz)
