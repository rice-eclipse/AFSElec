function OR = afs_import_ork(orkFile, simIndex)
%AFS_IMPORT_ORK Read geometry and simulation data from an OpenRocket .ork file.
%   OR = AFS_IMPORT_ORK(orkFile) parses the rocket design and the first
%   simulation stored in the file. OR = AFS_IMPORT_ORK(orkFile, k) uses
%   simulation k. Run the simulation in OpenRocket and save before
%   importing; the flight data is needed for thrust, mass, CG and inertia.
%
%   Supported layout: one stage built from a nose cone, body tubes and
%   transitions, with one trapezoidal fin set. All lengths are in m,
%   stations are measured from the nose tip.
%
%   OR.name, OR.motor            design and motor designation
%   OR.geo.*                     reference diameter, length, nose, fins
%   OR.sim.conditions.*          rail length/angle, wind, site
%   OR.sim.data                  table of the stored flight data
%   OR.sim.events                struct array of flight events (time, type)
%   OR.sim.summary               maxaltitude, timetoapogee, ... (OpenRocket's numbers)
if nargin < 2
    simIndex = 1;
end
txt = read_ork_text(orkFile);

%% Design
rocketTxt = element(txt, 'rocket', 1);
OR.name  = strtrim(child_text(rocketTxt, 'name'));
stageTxt = element(rocketTxt, 'stage', 1);
subTxt   = element(stageTxt, 'subcomponents', 1);
comps    = top_level_children(subTxt);

x = 0;                         % running station of the forward end
R = 0;
geo = struct('d', 0, 'L', 0, 'noseLength', 0, 'noseShape', '', 'fin', []);
for k = 1:numel(comps)
    c = comps(k);
    len = num(child_text(c.body, 'length'));
    switch c.tag
        case 'nosecone'
            geo.noseLength = len;
            geo.noseShape  = strtrim(child_text(c.body, 'shape'));
            R = max(R, num(child_text(c.body, 'aftradius')));
        case 'bodytube'
            R = max(R, num(child_text(c.body, 'radius')));
            finTxt = element(c.body, 'trapezoidfinset', 1);
            if ~isempty(finTxt)
                geo.fin = parse_fins(finTxt, x, len);
                motorTxt = element(c.body, 'motor', 1);
                if ~isempty(motorTxt)
                    OR.motor = strtrim(child_text(motorTxt, 'designation'));
                    OR.motorManufacturer = strtrim(child_text(motorTxt, 'manufacturer'));
                end
            end
        case 'transition'
            R = max(R, num(child_text(c.body, 'foreradius')), num(child_text(c.body, 'aftradius')));
    end
    if ~isnan(len)
        x = x + len;
    end
end
if isempty(geo.fin)
    error('afs_import_ork:noFins', 'No trapezoidal fin set found in %s.', orkFile);
end
if ~isfield(OR, 'motor')
    motorTxt = element(txt, 'motor', 1);
    OR.motor = strtrim(child_text(motorTxt, 'designation'));
    OR.motorManufacturer = strtrim(child_text(motorTxt, 'manufacturer'));
end
geo.d = 2*R;
geo.L = x;
OR.geo = geo;

%% Simulation
simTxt = element(txt, 'simulation', simIndex);
if isempty(simTxt)
    error('afs_import_ork:noSim', 'Simulation %d not found in %s.', simIndex, orkFile);
end
condTxt = element(simTxt, 'conditions', 1);
cond.railLength = num(child_text(condTxt, 'launchrodlength'));
% OpenRocket stores the rod angle/direction in degrees but wind direction in rad
cond.railAngle  = deg2rad(num(child_text(condTxt, 'launchrodangle')));     % rad from vertical
cond.railDir    = deg2rad(num(child_text(condTxt, 'launchroddirection'))); % rad, 0 = North
cond.windSpeed  = num(child_text(condTxt, 'windaverage'));
cond.windDir    = num(child_text(condTxt, 'winddirection'));       % rad
cond.windTurb   = num(child_text(condTxt, 'windturbulence'));
cond.siteAlt    = num(child_text(condTxt, 'launchaltitude'));
OR.sim.conditions = cond;

fd = regexp(simTxt, '<flightdata([^>]*)>', 'tokens', 'once');
OR.sim.summary = struct();
if ~isempty(fd)
    a = regexp(fd{1}, '(\w+)="([^"]*)"', 'tokens');
    for k = 1:numel(a)
        OR.sim.summary.(a{k}{1}) = num(a{k}{2});
    end
end

branch = regexp(simTxt, '<databranch[^>]*types="([^"]*)"[^>]*>(.*?)</databranch>', 'tokens', 'once');
if isempty(branch)
    error('afs_import_ork:noData', ['Simulation %d has no stored flight data. ' ...
        'Run it in OpenRocket and save the file.'], simIndex);
end
names = strtrim(strsplit(branch{1}, ','));
rows  = regexp(branch{2}, '<datapoint>([^<]*)</datapoint>', 'tokens');
M = zeros(numel(rows), numel(names));
for k = 1:numel(rows)
    M(k, :) = str2double(strsplit(rows{k}{1}, ','));
end
OR.sim.data = array2table(M, 'VariableNames', matlab.lang.makeValidName(names));
OR.sim.data.Properties.VariableDescriptions = names;
ev = regexp(branch{2}, '<event time="([^"]*)" type="([^"]*)"', 'tokens');
OR.sim.events = struct('time', cellfun(@(e) num(e{1}), ev, 'UniformOutput', false), ...
                       'type', cellfun(@(e) e{2}, ev, 'UniformOutput', false));
end

%% ------------------------------------------------------------------ helpers
function fin = parse_fins(t, xTube, tubeLen)
fin.n      = num(child_text(t, 'fincount'));
fin.root   = num(child_text(t, 'rootchord'));
fin.tip    = num(child_text(t, 'tipchord'));
fin.sweep  = num(child_text(t, 'sweeplength'));
fin.span   = num(child_text(t, 'height'));
fin.thickness = num(child_text(t, 'thickness'));
fin.cant   = num(child_text(t, 'cant'));
fin.rotation = num(child_text(t, 'rotation'));
m = regexp(t, '<axialoffset method="(\w+)">([^<]*)</axialoffset>', 'tokens', 'once');
off = num(m{2});
switch m{1}
    case 'top',    fin.xLE = xTube + off;
    case 'middle', fin.xLE = xTube + tubeLen/2 - fin.root/2 + off;
    case 'bottom', fin.xLE = xTube + tubeLen - fin.root + off;
    otherwise,     fin.xLE = off;   % absolute
end
end

function txt = read_ork_text(f)
% .ork files are usually a zip holding rocket.ork; very old ones are plain XML.
fid = fopen(f, 'r');
if fid < 0
    error('afs_import_ork:open', 'Cannot open %s.', f);
end
magic = fread(fid, 2, '*char').';
fclose(fid);
if strcmp(magic, 'PK')
    tmp = tempname;
    mkdir(tmp);
    cleanup = onCleanup(@() rmdir(tmp, 's'));
    files = unzip(f, tmp);
    isOrk = endsWith(files, '.ork', 'IgnoreCase', true);
    if ~any(isOrk)
        error('afs_import_ork:zip', 'No rocket.ork inside %s.', f);
    end
    txt = fileread(files{find(isOrk, 1)});
else
    txt = fileread(f);
end
end

function body = element(txt, tag, k)
% Inner text of the k-th <tag>...</tag> element (outermost match), or ''.
body = '';
[s, e] = regexp(txt, ['<' tag '(\s[^>]*)?(?<!/)>'], 'start', 'end');   % skip <tag/>
if numel(s) < k
    return;
end
from = e(k) + 1;
% walk forward, matching nested open/close tags of the same name
depth = 1;
pos = from;
pat = ['<(/?)' tag '(\s[^>]*)?(?<!/)>'];
while depth > 0
    [ms, me, tok] = regexp(txt(pos:end), pat, 'start', 'end', 'tokens', 'once');
    if isempty(ms)
        return;
    end
    if isempty(tok{1})
        depth = depth + 1;
    else
        depth = depth - 1;
    end
    if depth == 0
        body = txt(from:pos + ms - 2);
    end
    pos = pos + me;
end
end

function out = child_text(txt, tag)
% Text of the first <tag>...</tag> that is a direct child (not inside <subcomponents>).
out = '';
if isempty(txt)
    return;
end
sub = regexp(txt, '<subcomponents>', 'start', 'once');
if ~isempty(sub)
    txt = txt(1:sub - 1);
end
m = regexp(txt, ['<' tag '(\s[^>]*)?>([^<]*)</' tag '>'], 'tokens', 'once');
if ~isempty(m)
    out = m{2};
end
end

function comps = top_level_children(txt)
% Ordered list of the elements directly inside txt.
comps = struct('tag', {}, 'body', {});
pos = 1;
while true
    [s, tok, openTag] = regexp(txt(pos:end), '<(\w+)(\s[^>]*)?(?<!/)>', 'start', 'tokens', 'match', 'once');
    if isempty(s)
        break;
    end
    tag = tok{1};
    rest = txt(pos + s - 1:end);
    body = element(rest, tag, 1);
    comps(end + 1) = struct('tag', tag, 'body', body); %#ok<AGROW>
    % skip past this element's closing tag
    pos = pos + s - 1 + numel(openTag) + numel(body) + numel(['</' tag '>']);
end
end

function v = num(s)
% Number from an OpenRocket value; handles "auto 0.041" and missing values.
if isempty(s)
    v = NaN;
    return;
end
t = regexp(s, '[-+]?\d*\.?\d+([eE][-+]?\d+)?', 'match', 'once');
if isempty(t)
    v = NaN;
else
    v = str2double(t);
end
end
