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