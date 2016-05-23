function xmlParser()
clc;
filename = 'twolinkman.xml';
theStruct = XMLparse(filename);

names = {theStruct.Children.Name};
childrn = {theStruct.Children.Children};
wbIndx = strfind(names, 'worldbody');
wbIndx = find(not(cellfun('isempty', wbIndx)));
WB = childrn{wbIndx};

WBnames = {WB.Name};
WBchild = {WB.Children};
nRbts = strfind(WBnames, 'body');
nRbts = find(not(cellfun('isempty', nRbts)));

for r = 1:length(nRbts) %number of robots
    lnks = []; jnts = [];
    jIndx = strfind({WBchild{nRbts(r)}.Name}, 'joint');
    jIndx = find(not(cellfun('isempty', jIndx)));
    gIndx = strfind({WBchild{nRbts(r)}.Name}, 'geom');
    gIndx = find(not(cellfun('isempty', gIndx)));
    bIndx = strfind({WBchild{nRbts(r)}.Name}, 'body');
    bIndx = find(not(cellfun('isempty', bIndx)));
    childrn = {WBchild{nRbts(r)}.Children};
    attrib = {WBchild{nRbts(r)}.Attributes};
    if(~isempty(jIndx))
        rj = attrib{jIndx}; %root joint of each robot
    end
    if(~isempty(gIndx))
        rl = attrib{gIndx}; %root link of each robot
    end
    
    lnks(1).l = rl;
    jnts(1).j = rj;
    
    for l = 1:length(bIndx) %get all sublinks
        [sbLnks, sbJnts] = getAllSubMems(childrn{bIndx(l)});
  
        for i = 1:length(sbJnts)
            jnts(i+l).j = sbJnts(i).sj;
        end
        for i = 1:length(sbLnks)
           lnks(i+l).l = sbLnks(i).sl;
        end
    end
    rbts(r).lnks = lnks;
    rbts(r).jnts = jnts;
end

end

function [sbLnks,sbJnts] = getAllSubMems(body)
finish = 0; cntJ = 1; cntL = 1;
while(~finish)
jIndx = strfind({body.Name}, 'joint');
jIndx = find(not(cellfun('isempty', jIndx)));
gIndx = strfind({body.Name}, 'geom');
gIndx = find(not(cellfun('isempty', gIndx)));
bIndx = strfind({body.Name}, 'body');
bIndx = find(not(cellfun('isempty', bIndx)));
childrn = {body.Children};
attrib = {body.Attributes};
if(~isempty(jIndx))
    sbJnts(cntJ).sj = attrib{jIndx}; %root joint of each robot
    cntJ = cntJ+1;
end
if(~isempty(gIndx))
    sbLnks(cntL).sl = attrib{gIndx}; %root link of each robot
    cntL = cntL + 1;
end

if(isempty(bIndx))
    finish = 1;
else
    body = {childrn{bIndx}.Children};
end
end
end

function theStruct = XMLparse(filename)
% PARSEXML Convert XML file to a MATLAB structure.
try
   tree = xmlread(filename);
catch
   error('Failed to read XML file %s.',filename);
end

% Recurse over child nodes. This could run into problems 
% with very deeply nested trees.
try
   theStruct = parseChildNodes(tree);
catch
   error('Unable to parse XML file %s.',filename);
end

end

% ----- Local function PARSECHILDNODES -----
function children = parseChildNodes(theNode)
% Recurse over node children.
children = [];
if theNode.hasChildNodes
   childNodes = theNode.getChildNodes;
   numChildNodes = childNodes.getLength;
   allocCell = cell(1, numChildNodes);

   children = struct(             ...
      'Name', allocCell, 'Attributes', allocCell,    ...
      'Data', allocCell, 'Children', allocCell);

    for count = 1:numChildNodes
        theChild = childNodes.item(count-1);
        children(count) = makeStructFromNode(theChild);
    end
end
end

% ----- Local function MAKESTRUCTFROMNODE -----
function nodeStruct = makeStructFromNode(theNode)
% Create structure of node info.

nodeStruct = struct(                        ...
   'Name', char(theNode.getNodeName),       ...
   'Attributes', parseAttributes(theNode),  ...
   'Data', '',                              ...
   'Children', parseChildNodes(theNode));

if any(strcmp(methods(theNode), 'getData'))
   nodeStruct.Data = char(theNode.getData); 
else
   nodeStruct.Data = '';
end
end
% ----- Local function PARSEATTRIBUTES -----
function attributes = parseAttributes(theNode)
% Create attributes structure.

attributes = [];
if theNode.hasAttributes
   theAttributes = theNode.getAttributes;
   numAttributes = theAttributes.getLength;
   allocCell = cell(1, numAttributes);
   attributes = struct('Name', allocCell, 'Value', ...
                       allocCell);

   for count = 1:numAttributes
      attrib = theAttributes.item(count-1);
      attributes(count).Name = char(attrib.getName);
      attributes(count).Value = char(attrib.getValue);
   end
end
end