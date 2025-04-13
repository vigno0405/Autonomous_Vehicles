classdef ExampleHelperGazeboModel < handle
    %ExampleHelperGazeboModel - Class controlling individual models in Gazebo
    %   M = ExampleHelperGazeboModel(varargin) creates an object that allows for easy
    %   control and interaction with individual models in the Gazebo simulator.
    %   The constructor can take arguments denoting model name and where to
    %   retrieve it. 
    %
    %   ExampleHelperGazeboModel methods:
    %       addLink                 - Adds a link to the model
    %       addJoint                - Adds a joint to the model
    %       getModelXML             - Prints the SDF to a string
    %       saveLocalModel          - Writes SDF to a local file
    %
    %   ExampleHelperGazeboModel properties:
    %       Name                    - Name of the Model
    %       ModelObj                - DOM object describing the Model
    %       Links                   - Array of link structures
    %       Joints                  - Array of joint structures
    %
    %   See also ExampleHelperGazeboSpawnedModel, ExampleHelperGazeboCommunicator
    
    %   Copyright 2014 The MathWorks, Inc.
    
    properties (Access = public)
        Name = [];                % Name of the Model
        ModelObj = [];            % DOM object describing the Model
    end
    
    properties (SetAccess = protected)
        Links = [];               % Array of link structures
        Joints = [];              % Array of joint structures
    end
    
    methods
        function obj = ExampleHelperGazeboModel(varargin)
            %ExampleHelperGazeboModel Constructor
            
            % Determine model to retrieve and how to get it
            obj.Name = convertStringsToChars(varargin{1});
            if nargin == 1
                try
                    fileName = strcat(obj.Name, '.sdf');
                    obj.ModelObj = parseFile(matlab.io.xml.dom.Parser, fileName);
                catch
                    obj.ModelObj = matlab.io.xml.dom.Document('sdf');
                    doc = obj.ModelObj.getDocumentElement;
                    doc.setAttribute('version','1.4');
                    model = obj.ModelObj.createElement('model');
                    model.setAttribute('name', obj.Name);
                    doc.appendChild(model);
                    
                end
                
            elseif nargin == 2 && strcmpi(varargin{2},'gazeboDB')
                obj.ModelObj = matlab.io.xml.dom.Document('sdf');
                doc = obj.ModelObj.getDocumentElement;
                doc.setAttribute('version','1.4');
                model = obj.ModelObj.createElement('model');
                model.setAttribute('name', obj.Name);
                doc.appendChild(model);
                include = model.appendChild(obj.ModelObj.createElement('include'));
                uri = include.appendChild(obj.ModelObj.createElement('uri'));
                uri.appendChild(obj.ModelObj.createTextNode(['model://' obj.Name]));
            end
        end
        
        
        function linkstring = addLink(obj,linktype, param, varargin)
            %addLink - Adds a link to the model
            
            position = [0 0 0];
            orientation = [0 0 0];
            bounce = [];
            color = [];
            
            options = varargin(1:end);
            numOptions = numel(options)/2;
            for k = 1:numOptions
                opt = options{2*k-1};
                val = options{2*k};
                if strcmpi(opt,'bounce')
                    bounce = val;
                end
                if strcmpi(opt,'color')
                    color = val;
                end
                if strcmpi(opt,'position')
                    position = val;
                end
                if strcmpi(opt,'orientation')
                    orientation = val;
                end
            end
            
            inpose = [position orientation];
            
            % Creating the link for the model
            model = obj.ModelObj.getElementsByTagName('model');
            
            linknum = model.item(0).getElementsByTagName('link').getLength;
            link = model.item(0).appendChild(obj.ModelObj.createElement('link'));
            link.setAttribute('name', ['link' num2str(linknum)]);
            
            pose = link.appendChild(obj.ModelObj.createElement('pose'));
            pose.appendChild(obj.ModelObj.createTextNode(num2str(inpose)));
            
            % The collision aspect of the object
            collision = link.appendChild(obj.ModelObj.createElement('collision'));
            collision.setAttribute('name','collision');
            
            % The visual aspect of the object
            visual = link.appendChild(obj.ModelObj.createElement('visual'));
            visual.setAttribute('name','visual');
            collision.appendChild(getGeometry(obj,linktype,param));
            visual.appendChild(getGeometry(obj,linktype,param));
            
            if ~isempty(bounce)
                collision.appendChild(getBounce(obj,bounce));
            end
            
            if ~isempty(color)
                visual.appendChild(getColor(obj,color));
            end
            
            linkstring = struct('Name', char(link.getAttribute('name')), 'InitialPose', ...
                inpose);
            obj.Links = [obj.Links; linkstring];
            linkstring = linkstring.Name;
        end
        
        
        function jointstring = addJoint(obj, parent, child, type, limits, axis)
            %addJoint - Adds a joint to the model
            
            model = obj.ModelObj.getElementsByTagName('model');
            joint = model.item(0).appendChild(obj.ModelObj.createElement('joint'));
            jointnum = model.item(0).getElementsByTagName('joint').getLength;
            joint.setAttribute('name', ['joint' num2str(jointnum)]);
            joint.setAttribute('type', type);
            
            plink = joint.appendChild(obj.ModelObj.createElement('parent'));
            plink.appendChild(obj.ModelObj.createTextNode(parent));
            
            clink = joint.appendChild(obj.ModelObj.createElement('child'));
            clink.appendChild(obj.ModelObj.createTextNode(child));
            
            jaxis = joint.appendChild(obj.ModelObj.createElement('axis'));
            order = jaxis.appendChild(obj.ModelObj.createElement('xyz'));
            order.appendChild(obj.ModelObj.createTextNode(num2str(axis)));
            
            limit = jaxis.appendChild(obj.ModelObj.createElement('limit'));
            
            lower = limit.appendChild(obj.ModelObj.createElement('lower'));
            lower.appendChild(obj.ModelObj.createTextNode(num2str(limits(1))));
            upper = limit.appendChild(obj.ModelObj.createElement('upper'));
            upper.appendChild(obj.ModelObj.createTextNode(num2str(limits(2))));
            
            jointstring = struct('Name', char(joint.getAttribute('name')), 'Parent', ...
                parent, 'Child', child, 'Type', type, 'Limits', limits, ...
                'Axis', axis);
            obj.Joints = [obj.Joints; jointstring];
            
            jointstring = jointstring.Name;
        end
        
        function out = getModelXml(obj)
            %getModelXml - Prints the SDF to a string
            
            out = writeToString(matlab.io.xml.dom.DOMWriter, obj.ModelObj);

        end
        
        function saveLocalModel(obj, filename)
            %saveLocalModel - Writes SDF to a local file
            
            writeToFile(matlab.io.xml.dom.DOMWriter, obj.ModelObj, filename)
        end
        
        function out = getLinks(obj)
            out = obj.Links;
        end
        
        function out = getJoints(obj)
            out = obj.Joints;
        end
        
    end
    
    methods (Access = protected)
        
        function geom = getGeometry(obj,linktype,param)
            %getGeometry - Adds geometry element to the vision or collision tag
            
            geom = obj.ModelObj.createElement('geometry');
            type = geom.appendChild(obj.ModelObj.createElement(linktype));
            dim2 = [];
            switch linktype
                case 'sphere'
                    dim = type.appendChild(obj.ModelObj.createElement('radius'));
                case 'box'
                    dim = type.appendChild(obj.ModelObj.createElement('size'));
                case 'cylinder'
                    dim2 = type.appendChild(obj.ModelObj.createElement('radius'));
                    dim = type.appendChild(obj.ModelObj.createElement('length'));
            end
            dim.appendChild(obj.ModelObj.createTextNode(num2str(param(1))));
            if ~isempty(dim2)
                dim2.appendChild(obj.ModelObj.createTextNode(num2str(param(2))));
            end
        end
        
        function material = getColor(obj,color)
            %getColor - Creates element for the color of the link material
            
            material = obj.ModelObj.createElement('material');
            ambient = material.appendChild(obj.ModelObj.createElement('ambient'));
            diffuse = material.appendChild(obj.ModelObj.createElement('diffuse'));
            ambient.appendChild(obj.ModelObj.createTextNode(num2str(color)));
            diffuse.appendChild(obj.ModelObj.createTextNode(num2str(color)));
        end
        
        function surface = getBounce(obj,bounce)
            %getBounce - Creates element for the bounciness of the surface material
            
            surface = obj.ModelObj.createElement('surface');
            bouncetag = surface.appendChild(obj.ModelObj.createElement('bounce'));
            
            rcoeff = bouncetag.appendChild(obj.ModelObj.createElement('restitution_coefficient'));
            rcoeff.appendChild(obj.ModelObj.createTextNode(num2str(bounce(1))));
            
            threshold = bouncetag.appendChild(obj.ModelObj.createElement('threshold'));
            threshold.appendChild(obj.ModelObj.createTextNode(num2str(0)));
            
            contacttag = surface.appendChild(obj.ModelObj.createElement('contact'));
            ode = contacttag.appendChild(obj.ModelObj.createElement('ode'));
            maxvel = ode.appendChild(obj.ModelObj.createElement('max_vel'));
            maxvel.appendChild(obj.ModelObj.createTextNode(num2str(bounce(2))));
        end
    end
    
end