classdef SMORES < handle
%% The parameter of the module object
    properties (Access = private)
        Radius = 48;     % mm the radius of the wheel
        BoxEdge = 100;
        SquarePlainDisplacement = 50;
        CirclePlainDisplacement = 50;
        BendingAngleUpperLimitation = 90 ;  % The unit is degree
        BendingAngleLowerLimitation = -90;
        CenterDisMag = 25;
        RaiusOfMag = 10;
        DisplayOffSet = 0.5;
        WheelOffSet = 1;
        WheelRadius = 50;
        FrameWidth = 50;
        DistanceTwoWheel = 100;             %need to be modified
        BendingSpeed = 1;                   %unit:round per second   
        DiscTuringSpeed = 0.5;              %Turning Speed of the Top Disc
    end
%% Dynamics parameters
    properties
        CenterPos = [0,0,0]';   % The geomtry center of the SMORES module
        FaceNormVector = [0,0,-1]';
        AxisVector = [1,0,0]';
        PosDirVector = [0,1,0]'; % This vector has to perpendicular to AxisVector and only useful for planar motion
        BendingIdealAngle = 0;       % The bending Angle of the round dock, unit:degree
        BendingActualAngle = 0;
        Wheel1Angle = 0;        % The angle of the center line of the two green dots with horizontal, unit:degree
        Wheel2Angle = 0;
        DockIdealAngle = 0;          % The angle of the round dock parrellel with the axis of two wheels, unit:degree
        DockActualAngle = 0;
        Wheel1Speed = 0;        % The turning speed of the left wheel, n/s different direction by the sign of the value
        Wheel2Speed = 0;        % The turning speed of the right wheel
        ConnectFlag = cell(4,3);% Four faces can be connected and for each face, we need to know whether it's connected, if so connected to whom, which face
    end
%% public function
    methods
        function obj = SMORES(ctPosition,Wheel1,Wheel2,Dock,Bend,PosVector,MAxisVector,PositiveDirectionVector)
            obj.CenterPos = ctPosition;
            obj.BendingActualAngle = Bend;
            obj.BendingIdealAngle = Bend;
            obj.DockActualAngle = Dock;
            obj.DockIdealAngle = Dock;
            obj.Wheel1Angle = Wheel1;
            obj.Wheel2Angle = Wheel2;
            if nargin == 8
                if dot(PosVector,MAxisVector)==0
                    obj.FaceNormVector = PosVector/norm(PosVector);
                    obj.AxisVector = MAxisVector/norm(MAxisVector);
                else
                    display('The Two vectors of normal surface vector and Axis direction vector are not perpendicular, the system will keep the default value');
                end
                if dot(MAxisVector,PositiveDirectionVector)==0
                    obj.PosDirVector = PositiveDirectionVector/norm(PositiveDirectionVector);
                else
                    display('The Two vectors of Axis direction vector and postive driving direction are not perpendicular, the system will keep the default value');
                end
            end
        end
        function Wheel1SpeedControl(obj,Speed,Unit)   % Unit is the unit of the spped: 1. for deg/s 2. for rad/s 3. for n/s 4. for m/s
            switch Unit
                case 1
                    obj.Wheel1Speed = Speed/360;
                case 2
                    obj.Wheel1Speed = Speed/(2*pi);
                case 3
                    obj.Wheel1Speed = Speed;
                case 4
                    obj.Wheel1Speed = Speed/(2*pi*obj.Radius);
                otherwise
                    obj.Wheel1Speed = obj.Wheel1Speed;
                    display('The input speed unit is unclear, keep the original speed');
            end
%             Wheel1Spd = obj.Wheel1Speed;
        end
        function Wheel2SpeedControl(obj,Speed,Unit)   % Unit is the unit of the spped: 1. for deg/s 2. for rad/s 3. for n/s 4. for m/s
            switch Unit
                case 1
                    obj.Wheel2Speed = Speed/360;
                case 2
                    obj.Wheel2Speed = Speed/(2*pi);
                case 3
                    obj.Wheel2Speed = Speed;
                case 4
                    obj.Wheel2Speed = Speed/(2*pi*obj.Radius);
                otherwise
                    obj.Wheel2Speed = obj.Wheel2Speed;
                    display('The input speed unit is unclear, keep the original speed');
            end
        end
        function ModuleBendingAngle(obj,Angle)      % The input angle limit is -30~75 Acuator is super fast now
            if Angle > obj.BendingAngleUpperLimitation
                obj.BendingAngle = obj.BendingAngleUpperLimitation;
            elseif Angle < obj.BendingAngleLowerLimitation
                obj.BendingAngle = obj.BendingAngleLowerLimitation;
            else
                obj.BendingIdealAngle = Angle;
            end
        end
        function FrontDocAngle(obj,Angle)
            obj.DockIdealAngle = Angle;
        end
        function SpacePlot(obj)
            TheCenterPointSquare = obj.CenterPos+obj.FaceNormVector*obj.SquarePlainDisplacement;
            Edge1MidPoint = TheCenterPointSquare+obj.AxisVector*obj.BoxEdge/2;
            Edge2MidPoint = TheCenterPointSquare-obj.AxisVector*obj.BoxEdge/2;
            VerticalAxisVector = cross(obj.FaceNormVector,obj.AxisVector);
            BPoint1 = Edge1MidPoint+VerticalAxisVector*obj.BoxEdge/2;
            BPoint2 = Edge1MidPoint-VerticalAxisVector*obj.BoxEdge/2;
            BPoint3 = Edge2MidPoint-VerticalAxisVector*obj.BoxEdge/2;
            BPoint4 = Edge2MidPoint+VerticalAxisVector*obj.BoxEdge/2;
            Sq = [BPoint1,BPoint2,BPoint3,BPoint4];
            CenterOfBottomSquare = (BPoint1+BPoint3)/2+obj.FaceNormVector*obj.DisplayOffSet;
            BottomPad1 = CenterOfBottomSquare+obj.CenterDisMag*VerticalAxisVector;
            BottomPad2 = CenterOfBottomSquare-obj.CenterDisMag*VerticalAxisVector;
            BottomPad3 = CenterOfBottomSquare+obj.CenterDisMag*obj.AxisVector;
            BottomPad4 = CenterOfBottomSquare-obj.CenterDisMag*obj.AxisVector;
            fill3(Sq(1,:),Sq(2,:),Sq(3,:),'k');
            hold on
            plotCircle3Dgreen(BottomPad1',obj.FaceNormVector',obj.RaiusOfMag);
            plotCircle3Dgreen(BottomPad2',obj.FaceNormVector',obj.RaiusOfMag);
            plotCircle3Dred(BottomPad3',obj.FaceNormVector',obj.RaiusOfMag);
            plotCircle3Dred(BottomPad4',obj.FaceNormVector',obj.RaiusOfMag);
            M1Point1 = BPoint1 - obj.FaceNormVector*obj.SquarePlainDisplacement;
            M1Point2 = BPoint2 - obj.FaceNormVector*obj.SquarePlainDisplacement;
            M2Point1 = BPoint3 - obj.FaceNormVector*obj.SquarePlainDisplacement;
            M2Point2 = BPoint4 - obj.FaceNormVector*obj.SquarePlainDisplacement;
            Sd1 = [BPoint1 M1Point1 M1Point2 BPoint2];
            sd2 = [BPoint3 M2Point1 M2Point2 BPoint4];
            fill3(Sd1(1,:),Sd1(2,:),Sd1(3,:),'k','LineStyle','none');
            fill3(sd2(1,:),sd2(2,:),sd2(3,:),'k','LineStyle','none');
            Sd1 = [BPoint1 M1Point1 M2Point2 BPoint4];
            sd2 = [M1Point2 BPoint2 BPoint3 M2Point1];
            fill3(Sd1(1,:),Sd1(2,:),Sd1(3,:),'k');
            fill3(sd2(1,:),sd2(2,:),sd2(3,:),'k');
            plotCircle3D(((M1Point1+M1Point2)/2)',obj.AxisVector',obj.Radius);
            plotCircle3D(((M2Point1+M2Point2)/2)',obj.AxisVector',obj.Radius);
            TheCenterPointCircleNorm = -obj.FaceNormVector*cos(obj.BendingActualAngle*pi/180)+VerticalAxisVector*sin(obj.BendingActualAngle*pi/180);
            TheCenterPointCircle = obj.CenterPos+TheCenterPointCircleNorm*obj.CirclePlainDisplacement;
            plotCircle3D(TheCenterPointCircle',TheCenterPointCircleNorm',obj.Radius);
            CenterHight = TheCenterPointCircle+TheCenterPointCircleNorm*obj.DisplayOffSet;
            XVectorInTopCircle = (M1Point1-M2Point2)/norm(M1Point1-M2Point2);
            YVectorInTopCircle = cross(TheCenterPointCircleNorm,XVectorInTopCircle);
            Pad1Center = XVectorInTopCircle*cos(-obj.DockActualAngle/180*pi)*obj.CenterDisMag+YVectorInTopCircle*sin(-obj.DockActualAngle/180*pi)*obj.CenterDisMag+CenterHight;
            Pad2Center = XVectorInTopCircle*cos(pi-obj.DockActualAngle/180*pi)*obj.CenterDisMag+YVectorInTopCircle*sin(pi-obj.DockActualAngle/180*pi)*obj.CenterDisMag+CenterHight;
            Pad3Center = XVectorInTopCircle*cos(pi/2-obj.DockActualAngle/180*pi)*obj.CenterDisMag+YVectorInTopCircle*sin(pi/2-obj.DockActualAngle/180*pi)*obj.CenterDisMag+CenterHight;
            Pad4Center = XVectorInTopCircle*cos(-pi/2-obj.DockActualAngle/180*pi)*obj.CenterDisMag+YVectorInTopCircle*sin(-pi/2-obj.DockActualAngle/180*pi)*obj.CenterDisMag+CenterHight;
            plotCircle3Dgreen(Pad1Center',TheCenterPointCircleNorm',obj.RaiusOfMag);
            plotCircle3Dgreen(Pad2Center',TheCenterPointCircleNorm',obj.RaiusOfMag);
            plotCircle3Dred(Pad3Center',TheCenterPointCircleNorm',obj.RaiusOfMag);
            plotCircle3Dred(Pad4Center',TheCenterPointCircleNorm',obj.RaiusOfMag);
            LeftCenter = (M1Point1+M1Point2)/2;
            RightCenter = (M2Point1+M2Point2)/2;
            TopLeftCenter = LeftCenter+TheCenterPointCircleNorm*obj.CirclePlainDisplacement;
            TopRightCenter = RightCenter+TheCenterPointCircleNorm*obj.CirclePlainDisplacement;
            LeftPoint1 = LeftCenter+obj.FrameWidth/2*YVectorInTopCircle;
            LeftPoint2 = LeftCenter-obj.FrameWidth/2*YVectorInTopCircle;
            LeftPoint3 = TopLeftCenter-obj.FrameWidth/2*YVectorInTopCircle;
            LeftPoint4 = TopLeftCenter+obj.FrameWidth/2*YVectorInTopCircle;
            RightPoint1 = RightCenter+obj.FrameWidth/2*YVectorInTopCircle;
            RightPoint2 = RightCenter-obj.FrameWidth/2*YVectorInTopCircle;
            RightPoint3 = TopRightCenter-obj.FrameWidth/2*YVectorInTopCircle;
            RightPoint4 = TopRightCenter+obj.FrameWidth/2*YVectorInTopCircle;
            Sd1 = [LeftPoint1 LeftPoint2 LeftPoint3 LeftPoint4];
            sd2 = [RightPoint1 RightPoint2 RightPoint3 RightPoint4];
            sd3 = [LeftPoint3 LeftPoint4 RightPoint4 RightPoint3];
            fill3(Sd1(1,:),Sd1(2,:),Sd1(3,:),'k');
            fill3(sd2(1,:),sd2(2,:),sd2(3,:),'k');
            fill3(sd3(1,:),sd3(2,:),sd3(3,:),'k');
            LeftCenterWheel = (M1Point1+M1Point2)/2+obj.WheelOffSet*obj.AxisVector;
            RightCenterWheel = (M2Point1+M2Point2)/2-obj.WheelOffSet*obj.AxisVector;
            plotCircle3D(LeftCenterWheel',obj.AxisVector',obj.WheelRadius);
            plotCircle3D(RightCenterWheel',obj.AxisVector',obj.WheelRadius);
            LeftPadCenter  = (M2Point1+M2Point2)/2-(obj.WheelOffSet+obj.DisplayOffSet)*obj.AxisVector;  %(M1Point1+M1Point2)/2+(obj.WheelOffSet+obj.DisplayOffSet)*obj.AxisVector;
            RightPadCenter = (M1Point1+M1Point2)/2+(obj.WheelOffSet+obj.DisplayOffSet)*obj.AxisVector;  %(M2Point1+M2Point2)/2-(obj.WheelOffSet+obj.DisplayOffSet)*obj.AxisVector;
            LeftXVector = (M1Point1-M1Point2)/norm(M1Point1-M1Point2);
            LeftYVector = cross(obj.AxisVector,LeftXVector);
            LeftPad1 = LeftPadCenter+LeftXVector*cos(-obj.Wheel1Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(-obj.Wheel1Angle/180*pi)*obj.CenterDisMag;
            LeftPad2 = LeftPadCenter+LeftXVector*cos(pi-obj.Wheel1Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(pi-obj.Wheel1Angle/180*pi)*obj.CenterDisMag;
            LeftPad3 =  LeftPadCenter+LeftXVector*cos(pi/2-obj.Wheel1Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(pi/2-obj.Wheel1Angle/180*pi)*obj.CenterDisMag;
            LeftPad4 =  LeftPadCenter+LeftXVector*cos(-pi/2-obj.Wheel1Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(-pi/2-obj.Wheel1Angle/180*pi)*obj.CenterDisMag;
            RightPad1 = RightPadCenter+LeftXVector*cos(-obj.Wheel2Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(-obj.Wheel2Angle/180*pi)*obj.CenterDisMag;
            RightPad2 = RightPadCenter+LeftXVector*cos(pi-obj.Wheel2Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(pi-obj.Wheel2Angle/180*pi)*obj.CenterDisMag;
            RightPad3 = RightPadCenter+LeftXVector*cos(pi/2-obj.Wheel2Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(pi/2-obj.Wheel2Angle/180*pi)*obj.CenterDisMag;
            RightPad4 = RightPadCenter+LeftXVector*cos(-pi/2-obj.Wheel2Angle/180*pi)*obj.CenterDisMag+LeftYVector*sin(-pi/2-obj.Wheel2Angle/180*pi)*obj.CenterDisMag;
            plotCircle3Dgreen(LeftPad1',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dgreen(LeftPad2',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dred(LeftPad3',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dred(LeftPad4',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dgreen(RightPad1',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dgreen(RightPad2',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dred(RightPad3',obj.AxisVector',obj.RaiusOfMag);
            plotCircle3Dred(RightPad4',obj.AxisVector',obj.RaiusOfMag);
            grid on;
            box on;
            axis([-200 200 -200 200 -200 200])
            axis vis3d;
%             hold off;
        end
        function MotionCalculation(obj,Tinterval)
            % On the ground and free drive
            if obj.AxisVector(3) == 0 &&  obj.CenterPos(3)==0     % on the ground will have planar motion
                if obj.Wheel1Speed == obj.Wheel2Speed   % The Truning radius is going to infinity
                    obj.CenterPos = obj.CenterPos+obj.Wheel1Speed*(2*pi*obj.Radius)*Tinterval*obj.PosDirVector;
                else
                    EquiOmega = abs(obj.Wheel2Speed-obj.Wheel1Speed)/obj.DistanceTwoWheel*(2*pi*obj.Radius);
                    TurnRadius = (obj.Wheel2Speed+obj.Wheel1Speed)/(obj.Wheel2Speed-obj.Wheel1Speed)*obj.DistanceTwoWheel/2;
                    RotationAngle = EquiOmega*Tinterval;
                    RotationMatrix = [cos(RotationAngle) -sin(RotationAngle);sin(RotationAngle) cos(RotationAngle)];
                    DisplaceInDir = abs(TurnRadius)*sin(RotationAngle);
                    DisplaceInPer = abs(TurnRadius)-abs(TurnRadius)*cos(RotationAngle);
                    if obj.Wheel2Speed+obj.Wheel1Speed>=0   % Calculate the position of each module
                        if TurnRadius>0
                            obj.CenterPos = obj.CenterPos+DisplaceInDir*obj.PosDirVector-DisplaceInPer*obj.AxisVector;
                        else
                            obj.CenterPos = obj.CenterPos+DisplaceInDir*obj.PosDirVector+DisplaceInPer*obj.AxisVector;
                        end
                    else
                        if TurnRadius>0
                            obj.CenterPos = obj.CenterPos-DisplaceInDir*obj.PosDirVector-DisplaceInPer*obj.AxisVector;
                        else
                            obj.CenterPos = obj.CenterPos-DisplaceInDir*obj.PosDirVector+DisplaceInPer*obj.AxisVector;
                        end
                    end
                    obj.PosDirVector(1:2) = RotationMatrix*obj.PosDirVector(1:2);
                    obj.AxisVector(1:2) = RotationMatrix*obj.AxisVector(1:2);
                    obj.FaceNormVector(1:2) = RotationMatrix*obj.FaceNormVector(1:2);
                end
                obj.Wheel1Angle = obj.Wheel1Angle+obj.Wheel1Speed*360*Tinterval;
                obj.Wheel2Angle = obj.Wheel2Angle+obj.Wheel2Speed*360*Tinterval;
            else    % in the air, only rotating
                obj.Wheel1Angle = obj.Wheel1Angle+obj.Wheel1Speed*360*Tinterval;
                obj.Wheel2Angle = obj.Wheel2Angle+obj.Wheel2Speed*360*Tinterval;
            end
            if obj.BendingActualAngle~=obj.BendingIdealAngle
                if obj.BendingActualAngle>obj.BendingIdealAngle
                    if obj.BendingActualAngle-obj.BendingIdealAngle>obj.BendingSpeed*360*Tinterval
                        obj.BendingActualAngle=obj.BendingActualAngle-obj.BendingSpeed*360*Tinterval;
                    else
                        obj.BendingActualAngle=obj.BendingIdealAngle;
                    end
                else
                    if obj.BendingIdealAngle-obj.BendingActualAngle>obj.BendingSpeed*360*Tinterval
                        obj.BendingActualAngle=obj.BendingActualAngle+obj.BendingSpeed*360*Tinterval;
                    else
                        obj.BendingActualAngle=obj.BendingIdealAngle;
                    end
                end
            else    % Bending first, and then turning
                if obj.DockIdealAngle~=obj.DockActualAngle
                    if obj.DockIdealAngle>obj.DockActualAngle
                        if obj.DockIdealAngle-obj.DockActualAngle>obj.DiscTuringSpeed*360*Tinterval
                            obj.DockActualAngle=obj.DockActualAngle+obj.DiscTuringSpeed*360*Tinterval;
                        else
                            obj.DockActualAngle=obj.DockIdealAngle;
                        end
                    else
                        if obj.DockActualAngle-obj.DockIdealAngle>obj.DiscTuringSpeed*360*Tinterval
                            obj.DockActualAngle=obj.DockActualAngle+obj.DiscTuringSpeed*360*Tinterval;
                        else
                            obj.DockActualAngle=obj.DockIdealAngle;
                        end
                    end
                end
            end
        end
        function [FacVec,FacPos] = ConnectionCheck(obj,WhichFace) % Return the face vector and face center point
            
        end
    end    
end