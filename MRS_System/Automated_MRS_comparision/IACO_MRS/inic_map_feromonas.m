function [mapa_fer] = inic_map_feromonas(mapa_obs)
[n,m]=size(mapa_obs);
mapa_fer = mapa_obs; % Inicializar el mapa de feromonas como una copia
    for i=1:n
        for j=1:m
            if mapa_obs(i,j)==1
                mapa_fer(i,j)=viabilidad(i,j,mapa_obs);
                
            end
        end
    end
end

function [value]=viabilidad(i,j,mapa)%Inicializacion de feromonas
card=0;
[n,m]=size(mapa);
    if(i==1 || i==n || j==1 || j==m)
        value=2;
    else
        for k=max(1,i-1):min(i+1,n)
            for l=max(1,j-1):min(j+1,m)
                if mapa(k,l)==1 %Condicion de espacio libre
                    card=card+1;
                end
            end
        end
        value=10*(card-1)/8;
    end
end

