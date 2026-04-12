function upObs=updateObstacles(Robots,obstacles)
n=length(obstacles);
for i =1:length(Robots)
    if length(Robots(i).obs)>n %hay algun objeto virtual ya presente
        for j=n+1:length(Robots(i).obs)
            newVirtualObs=true; %presuponemos que es cierto
            for k=n:length(obstacles) %comparamos con los objetos viruales ya almacenados
                if(isequal(obstacles(k),Robots(i).obs(j)))
                    newVirtualObs=false;
                end
            end
            
            if newVirtualObs
                obstacles(length(obstacles)+1)=Robots(i).obs(j);
            end
        
        end
    end
end

upObs=obstacles;


end