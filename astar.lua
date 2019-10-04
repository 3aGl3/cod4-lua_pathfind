-- ======================================================================
-- Copyright (c) 2012 RapidFire Studio Limited
-- All Rights Reserved.
-- http://www.rapidfirestudio.com

-- Permission is hereby granted, free of charge, to any person obtaining
-- a copy of this software and associated documentation files (the
-- "Software"), to deal in the Software without restriction, including
-- without limitation the rights to use, copy, modify, merge, publish,
-- distribute, sublicense, and/or sell copies of the Software, and to
-- permit persons to whom the Software is furnished to do so, subject to
-- the following conditions:

-- The above copyright notice and this permission notice shall be
-- included in all copies or substantial portions of the Software.

-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
-- EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
-- MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
-- IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
-- CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
-- TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
-- SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
-- ======================================================================

local astar = {}

----------------------------------------------------------------
-- local variables
----------------------------------------------------------------

local INF = 1/0
local cachedPaths = nil
local nodes = nil
local cachedNum = 0

----------------------------------------------------------------
-- local functions
----------------------------------------------------------------

local function VectorLengthSquared ( vec3 )

	return ( ( vec3 [ 1 ] * vec3 [ 1 ] ) + ( vec3 [ 2 ] * vec3 [ 2 ] ) + ( vec3 [ 3 ] * vec3 [ 3 ] ) )
end

local function VectorSubtract ( v1, v2 )

	local vec3 = {}
	vec3 [ 1 ] = v1 [ 1 ] - v2 [ 1 ]
	vec3 [ 2 ] = v1 [ 2 ] - v2 [ 2 ]
	vec3 [ 3 ] = v1 [ 3 ] - v2 [ 3 ]
	return vec3
end

local function Vec3DistanceSq ( v1, v2 )

	local vec3 = VectorSubtract ( v1, v2 )
	return VectorLengthSquared ( vec3 )
end

local function dist_between ( nodeA, nodeB )

	return Vec3DistanceSq ( nodeA.origin, nodeB.origin )
end

local function heuristic_cost_estimate ( nodeA, nodeB )

	return Vec3DistanceSq ( nodeA.origin, nodeB.origin )
end

local function lowest_f_score ( set, f_score )

	local lowest, bestNode = INF, nil
	for _, node in ipairs ( set ) do
		local score = f_score [ node ]
		if score < lowest then
			lowest, bestNode = score, node
		end
	end
	return bestNode
end


local function neighbor_nodes ( theNode )

	local neighbors = {}
	for _, nodenum in ipairs ( theNode.children ) do
			table.insert ( neighbors, nodes [ nodenum ] )
	end
	return neighbors
end

local function not_in ( set, theNode )

	for _, node in ipairs ( set ) do
		if node == theNode then return false end
	end
	return true
end

local function remove_node ( set, theNode )

	for i, node in ipairs ( set ) do
		if node == theNode then
			set [ i ] = set [ #set ]
			set [ #set ] = nil
			break
		end
	end
end

local function unwind_path ( flat_path, map, current_node )

	if map [ current_node ] then
		table.insert ( flat_path, 1, map [ current_node ] )
		return unwind_path ( flat_path, map, map [ current_node ] )
	else
		return flat_path
	end
end

local function buildNodes ( tokens )

	local protonode = {}
	protonode.id = tonumber ( tokens[ 1 ] [ 1 ] )
	protonode.origin = {}
	protonode.children = {}

	for _, org in ipairs ( tokens [ 2 ] ) do
		table.insert ( protonode.origin, tonumber ( org ) )
	end

	for _, child in ipairs ( tokens [ 3 ] ) do
		local realchild = tonumber( child )
		if realchild ~= protonode.id then
			table.insert ( protonode.children, realchild )
		end
	end

	table.insert ( nodes, protonode )
end

---Clears all pathfinding data
local function clearAll ()
	-- clear navmesh/waypoints
	nodes = {}
	verts = {}

	-- clear cache
	cachedNum = 0
	cachedPaths = {}

	collectgarbage()
end		-- clearAll

----------------------------------------------------------------
-- pathfinding functions
----------------------------------------------------------------

local function a_star ( start, goal )

	local closedset = {}
	local openset = { start }
	local came_from = {}

	local g_score, f_score = {}, {}
	g_score [ start ] = 0
	f_score [ start ] = heuristic_cost_estimate ( start, goal )

	while #openset > 0 do

		local current = lowest_f_score ( openset, f_score )
		if current == goal then
			local path = unwind_path ( {}, came_from, goal )
			table.insert ( path, goal )
			return path
		end

		remove_node ( openset, current )
		table.insert ( closedset, current )

		local neighbors = neighbor_nodes ( current )
		for _, neighbor in ipairs ( neighbors ) do
			if not_in ( closedset, neighbor ) then

				local tentative_g_score = g_score [ current ] + dist_between ( current, neighbor )

				if not_in ( openset, neighbor ) or tentative_g_score < g_score [ neighbor ] then
					came_from 	[ neighbor ] = current
					g_score 	[ neighbor ] = tentative_g_score
					f_score 	[ neighbor ] = g_score [ neighbor ] + heuristic_cost_estimate ( neighbor, goal )
					if not_in ( openset, neighbor ) then
						table.insert ( openset, neighbor )
					end
				end
			end
		end
	end

	return nil -- no valid path
end

----------------------------------------------------------------
-- exposed functions
----------------------------------------------------------------

---Loads the given waypoints/navmesh
function astar.loadNodes( newNodes, newVerts )
	-- check if we got new nav nodes
	if not newNodes then
		Plugin_Scr_Error( "astar.loadNodes: Got no new nodes.\n" )
		return
	end

	-- clear all pathfinding data
	clearAll()

	-- save the new nav vertices, if we have any
	if newVerts then
		for i,v in ipairs(newVerts) do
			-- check if the vertex is valid
			if #v == 3 then
				verts[i] = v
			else
				Plugin_Scr_Error( "astar.loadNodes: Invalid vertex ".. i ..".\n" )
			end
		end
	end

	-- save the new nav nodes
	for i,n in ipairs(newNodes) do
		-- check if all required data is present
		if n.id and n.origin and n.children then
			-- check if required data is valid
			if #n.origin == 3 then
				nodes[n.id] = n
			else
				Plugin_Scr_Error( "astar.loadNodes: Invalid node ".. i ..".\n" )
			end

			-- TODO possibly check if children and veritces are valid
		end
	end

	return true
end		-- astar.loadNodes

---Returns the number of cached paths
function astar.cacheDebug ()
	return cachedNum
end		-- astar.cacheDebug

---Returns the number of nav nodes and vertices
function astar.meshDebug()
	return #nodes, #verts
end		-- astar.meshDebug

---Returns the path from start to goal
function astar.path( start, goal )
	-- check if variables are valid
	if not start or not goal or not nodes[start] or not nodes[goal] then
		return
	end

	-- check if the path is already cached
	local isCached = true
	if not cachedPaths[start] then
		cachedPaths[start] = {}
		isCached = false
	elseif cachedPaths[start][goal] then
		return cachedPaths[start][goal]
	end

	-- check for cached children of start node
	for _, child in ipairs( nodes[start].children ) do
		if cachedPaths[child] and cachedPaths[child][goal] then
			return child
		end
	end

	-- Check for cached children of goal node
	if isCached then
		for _, child in ipairs ( nodes [ goal ].children ) do
			if cachedPaths [ start ] [ child ] then
				return cachedPaths [ start ] [ child ]
			end
		end
	end

	local resPath = a_star ( nodes[ start ], nodes[ goal ] )
	if not resPath then
		return nil
	end

	local paths = {}
	for _, node in ipairs ( resPath ) do
		table.insert ( paths, node.id )
	end

	for i, id in ipairs ( paths ) do
		if id == goal then
			break
		end

		if not cachedPaths [ id ] then
			cachedPaths [ id ] = {}
		end
		if not cachedPaths [ id ] [ goal ] then
			local n = i + 1
			cachedPaths [ id ] [ goal ] = paths [ n ] -- Cache only next node
			cachedNum = cachedNum + 1
		end
	end

	return resPath[ 2 ].id
end		-- astar.path

local function tcopy( in_table )
	local out_table = {}
	for k,v in pairs(in_table) do
		if type(v) == "table" then
			out_table[k] = tcopy(v)
		else
			out_table[k] = v
		end
	end
	return out_table
end

-- Returns true if the given point is in the given polygon
function astar.isPointInPoly( origin, pid )
	local function crossProdTest( a, b, c )
		if a[2] == b[2] and a[2] == c[2] then
			if (b[1] <= a[1] and a[1] <= c[1]) or (c[1] <= a[1] <= b[1]) then
				return 0
			else
				return 1
			end
		end

		if b[2] > c[2] then		-- swap b and c
			local t = b
			b = c
			c = t
		end

		if a[2] == b[2] and a[1] == b[1] then
			return 0
		end

		if a[2] <= b[2] or a[2] > c[2] then
			return 1
		end

		local delta = (b[1]-a[1]) * (c[2]-a[2]) - (b[2]-a[2]) * (c[1]-a[1])

		if delta > 0 then
			return -1
		elseif delta < 0 then
			return 1
		else
			return 0
		end
	end		-- crossProdTest

	local lo		-- lowest point in the polygon
	local hi		-- highest point in the polygon

	local poly = nodes[pid]
	local v = tcopy(poly.verts);
	local passes = #v;
	v[passes+1] = v[1];
	local t = -1;

--	Plugin_Printf ( "isPointInPoly, Checking for ".. pid ..", verts ".. #v.."/".. passes ..".\n" )

	-- go through the vertices of the polygon and check them
	for i=1, passes, 1 do
	--	Plugin_Printf ( "crossProdTest, Checking for ".. i ..": ".. v[i] .." and ".. v[i+1] ..".\n" )
		t = t * crossProdTest( origin, verts[v[i]], verts[v[i+1]] )

		if not lo or verts[v[i]][3] < lo then
			lo = verts[v[i]][3]
		end

		if not hi or verts[v[i]][3] > hi then
			hi = verts[v[i]][3]
		end
	end

	-- return false if the point is above or below the polygon
	if origin[3] < lo-18 then
		return false
	end

	if origin[3] > hi+18+(poly.height or 0) then
		return false
	end

	-- return if the point is within the boundaries of the polygon
	if t >= 0 then
		return true
	else
		return false
	end
end		-- astar.isPointInPoly

-- Returns the nav poly at the given origin
function astar.getNavPoly( origin )
	for nid, node in ipairs( nodes ) do
		if astar.isPointInPoly( origin, node.id ) then
			return node.id
		end
	end
end		-- astar.getNavPoly

---Gets the waypoint closest to the given origin
function astar.getWaypoint( origin )
	local nearestWp = -1
	local nearestDist
	local dist = nil

	for _, node in ipairs( nodes ) do
		dist = Vec3DistanceSq( origin, node.origin )

		if not nearestDist or dist < nearestDist then
			nearestDist = dist
			nearestWp = node.id
		end
	end

	return nearestWp
end		-- astar.getWaypoint

return astar
