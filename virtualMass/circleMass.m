classdef circleMass<handle
    properties
        centre;
        radius;
        fig;
        axs;
        obj;
    end
    methods
        function this=circleMass(centre,radius)
            this.centre=centre;
            this.radius=radius;
        end
        
        function modifyCircle(circleMass,centre,radius)
            circleMass.centre=centre;
            if ~radius
                radius=circleMass.radius;
            end
            circleMass.radius=radius;
            x=centre(1);
            y=centre(2);
            circleMass.obj=set(circleMass.obj, 'Position', ...
            [x-circleMass.radius,y-circleMass.radius,2*circleMass.radius, ...
             2*circleMass.radius]);
        end
        
        %% Next two functions are for animations only
        function plotCircle(circleMass,fig,axs)
            if ~exist('fig','var')
                circleMass.fig=figure();
            else 
                circleMass.fig=fig;
            end
            if ~exist('axs','var')
                    circleMass.axs=gca;
            else
                circleMass.axs=axs;
            end
            x=circleMass.centre(1);
            y=circleMass.centre(2);
            circleMass.obj=rectangle('Position',[x-circleMass.radius, ...
                y-circleMass.radius,2*circleMass.radius, ...
                2*circleMass.radius],'Curvature',[1 1],'FaceColor',...
                [0 .5 .5]);
%             ang=0:0.01:2*pi; 
%             xp=circleMass.radius*cos(ang);
%             yp=circleMass.radius*sin(ang);
%             obj=plot(circleMass.centre(1)+xp,circleMass.centre(2)+yp,'Parent', circleMass.axs);
            
        end
        
        function updateState(circleMass,centre)
            circleMass.centre=centre;
            x=centre(1);
            y=centre(2);
%             circleMass.obj=rectangle('Position',[x-circleMass.radius, ...
%                 y-circleMass.radius,2*circleMass.radius, ...
%                 2*circleMass.radius],'Curvature',[1 1],'FaceColor',...
%                 [0 .5 .5]);
            set(circleMass.obj, 'Position', ...
            [x-circleMass.radius,y-circleMass.radius,2*circleMass.radius, ...
             2*circleMass.radius]);
        end
            
    end
end
        
