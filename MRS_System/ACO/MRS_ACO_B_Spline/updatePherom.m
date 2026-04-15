function [newPherom]=updatePherom(oldPherom,ants,rho)
    Q=500;%Valor update feromonas
    newPherom=(1-rho)*oldPherom;%Proceso de evaporacion
    %Deposicion de feromonas en aquellas celdas por las que hayan pasado
    %las hormigas
    for i=1:size(ants,1)
        path_i=ants(i).path;
        for j=1:size(path_i,1)
            x=path_i(j,1);
            y=path_i(j,2);
            newPherom(x,y)=newPherom(x,y)+Q/ants(i).cost;
        end

    end
      %%Control de valores maximos y minimos
      newPherom=max(newPherom,0);%Valor minimo de feromonas es 0 para evitar valores negativos
      newPherom=min(newPherom,30);%Valores ha ajustar de las feromonas 
      %Existe un compormiso para la convergencia entre rho y los valores
      %maximos que permitimos de feromonas

end