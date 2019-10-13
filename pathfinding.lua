-- ======================================================================
-- Provides multiple functions for pathfinding for CoD4X
-- ======================================================================

-- Include the AStar algorithm
local astar = require( "lua/astar" )
local navmesh = false

-- Add GSC Functions
Plugin_ScrAddFunction( "AStarSearch" )				-- nextNode = AStarSearch( currentNode, targetNode )
Plugin_ScrAddFunction( "LoadNavmeshInternal" )		-- LoadNavmeshInternal()
Plugin_ScrAddFunction( "LoadWaypointsInternal" )	-- LoadWaypointsInternal()
Plugin_ScrAddFunction( "GetNavNode" )				-- node = GetNavNode( origin )
Plugin_ScrAddFunction( "IsPointInPoly" )			-- in_poly = IsPointInPoly( origin, poly_id )

---Seperates the given string by the given delimiter
local function strtok( inputstr, sep )
	local t = {}
	for str in string.gmatch ( inputstr, "([^"..sep.."]+)" ) do
		table.insert ( t, str )
	end

	return t
end		-- strtok

---Loads the waypoints for the current map
function LoadWaypointsInternal()
	local nodes = {}		-- all nodes

	-- get the filename for the csv
	local fname = Plugin_Scr_GetString( 0 )

	-- attempt to load the given file
	local file, error = io.open( fname )
	if error then
		Plugin_Scr_Error( "LoadWaypointsInternal: Unable to load waypoints from '".. fname .."', Error: ".. error .."\n" )
		return
	end

	-- read each line of the file
	while true do
		local line = file:read ()
		if not line then
			break
		end

		-- seperate the line into the different fields
		local tokens = strtok( line, "," )

		-- create the nav node
		local node = {}
		node.children = {}

		-- get the node id, first field
		node.id = tonumber( tokens[1] )

		-- get the node origin, second field
		local origin = strtok( tokens[2], "%s" )
		node.origin = { tonumber(origin[1]), tonumber(origin[2]), tonumber(origin[3]) }

		-- get the node children
		for i,child in ipairs( strtok(tokens[3], "%s") ) do
			table.insert( node.children, tonumber(child) )
		end

	--	Plugin_Printf( "Loaded waypoint ".. node.id .." at (".. node.origin[1] ..",".. node.origin[2] ..",".. node.origin[3] .."), with ".. #node.children .." children.\n" )

		-- save the node into the nodes table
		table.insert( nodes, node )
	end

	-- close the file
	file:close()

	-- load the nodes into astar
	if astar.loadNodes( nodes ) then
		Plugin_Printf( "LoadWaypointsInternal: Loaded ".. #nodes .." waypoints from csv file.\n" )
	else
		Plugin_Scr_Error( "LoadWaypointsInternal: Could not load waypoints.\n" )
	end
end		-- LoadWaypointsInternal

---Loads the navmesh for the current map
function LoadNavmeshInternal()
	-- set the navmesh flag
	navmesh = true

	local nodes = {}		-- all nodes
	local verts = {}		-- all verts

	-- get the filename for the csv
	local fname = Plugin_Scr_GetString( 0 )

	-- attempt to load the given file
	local file, error = io.open( fname )
	if error then
		Plugin_Scr_Error( "LoadNavmeshInternal: Unable to load navmesh from '".. fname ..", Error: ".. error .."\n" )
		return
	end

	-- read each line of the file
	while true do
		local line = file:read ()
		if not line then
			break
		end

		-- seperate the line into the different fields
		local tokens = strtok( line, "," )

		if tokens[1] == "v" then
			-- get the origin and save it as the id
			local origin = strtok( tokens[3], "%s" )
			verts[tonumber(tokens[2])] = { tonumber(origin[1]), tonumber(origin[2]), tonumber(origin[3]) }
		elseif tokens[1] == "p" then
			-- create the nav node
			local node = {}
			node.verts = {}
			node.children = {}

			-- get the node id, first field
			node.id = tonumber( tokens[2] )

			-- get the node origin, second field
			local origin = strtok( tokens[3], "%s" )
			node.origin = { tonumber(origin[1]), tonumber(origin[2]), tonumber(origin[3]) }

			-- get the node children
			for i,child in ipairs( strtok(tokens[4], "%s") ) do
				table.insert( node.children, tonumber(child) )
			end

			node.type = tokens[5] ~= "nil" and tokens[5]

			node.height = tokens[6] ~= "nil" and tonumber(tokens[6])

			-- get the node vertices
			for i,vert in ipairs( strtok(tokens[7], "%s") ) do
				table.insert( node.verts, tonumber(vert) )
			end

		--	Plugin_Printf( "Loaded navpoly ".. node.id .." at (".. node.origin[1] ..",".. node.origin[2] ..",".. node.origin[3] .."), with ".. #node.children .." children and ".. #node.verts .." vertices.\n" )

			-- save the node into the nodes table
			table.insert( nodes, node )
		else
			Plugin_Scr_Error( "LoadNavmeshInternal: Invalid line in csv file.\n" )
		end
	end

	-- close the file
	file:close()

	-- load the nodes into astar
	if astar.loadNodes( nodes, verts ) then
		Plugin_Printf( "LoadNavmeshInternal: Loaded ".. #nodes .." polys, with ".. #verts .." vertices from csv file.\n" )
	else
		Plugin_Scr_Error( "LoadNavmeshInternal: Could not load navmesh.\n" )
	end
end		-- LoadNavmeshInternal

---Returns true if the given point is inside the given polygon
function IsPointInPoly()
	if not navmesh then
		return false
	end

	local vec3 = Plugin_Scr_GetVector( 0 )
	local node = Plugin_Scr_GetInt( 0 )

	local in_poly = astar.isPointInPoly( vec3, node+1 )
	Plugin_Scr_AddBool( in_poly )
end

---Returns the nav node for the given origin
function GetNavNode()
	local nearestNode
	local vec3 = Plugin_Scr_GetVector( 0 )

	if navmesh then
		nearestNode = astar.getNavPoly( vec3 )
		if not nearestNode then
			Plugin_Printf( "GetNavNode: Unable to find nav node at (".. vec3[1]..","..vec3[2]..",".. vec3[3] ..").\n" )
			nearestNode = 0
		end
	else
		nearestNode = astar.getWaypoint( vec3 )
	end

	Plugin_Scr_AddInt( nearestNode - 1 )
end

---Returns the next node in the path from one node to another
function AStarSearch()

	local start = Plugin_Scr_GetInt ( 0 )
	local goal = Plugin_Scr_GetInt ( 1 )
	local result = astar.path ( start + 1, goal + 1 )
	if not result then
		result = 0
		Plugin_Printf( "AStarSearch: Unable to find path" )
	end

	Plugin_Scr_AddInt( result - 1 )
end		-- AStarSearch

-- Add console commands
Plugin_AddCommand( "pfind_cache_debug", 0 )
Plugin_AddCommand( "pfind_nmesh_debug", 0 )
Plugin_AddCommand( "pfind_inpoly", 0 )

---Prints the number of cached paths to the console
function pfind_cache_debug()
	local num = astar.cacheDebug ()
	Plugin_Printf ( "There are currently " .. num .. " cached paths.\n" )
end		-- pfind_cache_debug

---Prints details about the navmesh/waypoints to the console
function pfind_nmesh_debug()
	local nodes, verts = astar.meshDebug()
	Plugin_Printf ( "There are currently " .. nodes .. " nav nodes and ".. verts .. " vertices.\n" )
end

---Checks in what poly the origin is
function pfind_inpoly()
	local vec3 = {1120.0, 639.0, 24.0}
	local poly = astar.getNavPoly( vec3 )
	Plugin_Printf ( "The point (".. vec3[1] ..",".. vec3[2] ..",".. vec3[3] .. ") is in poly ".. poly ..".\n" )
end
