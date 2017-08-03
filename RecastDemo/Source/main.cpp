//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <vector>
#include <string>

#include "imgui.h"
#include "imguiRenderGL.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"
#include "DetourNavMeshBuilder.h"

#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};
Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }
static SampleItem g_samples[] =
{
	{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" },
	{ createTempObstacle, "Temp Obstacles" },
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

rcConfig* loadConfigFile(const std::string configFilePath)
{
	rcConfig *buf;
	FILE* fp = fopen(configFilePath.c_str(), "rb");
	if (!fp)
	{
		return buf;
	}
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return buf;
	}

	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return buf;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return buf;
	}
	buf = new rcConfig();
	if (!buf)
	{
		fclose(fp);
		return buf;
	}
	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);
	if (readLen != 1)
	{
		delete[] buf;
		return buf;
	}
	return buf;
}

bool buildFromOutSise(int m_partitionType, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb, const std::string indexsFilePath, const std::string vertexsFilePath, const std::string configFilePath, const std::string outputFilePath)
{
	InputGeom *m_geom = new InputGeom;
	BuildContext *m_ctx;
	if (!m_geom->loadFromVector(m_ctx, indexsFilePath, vertexsFilePath))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	bool m_keepInterResults = false;
	rcConfig *m_cfg = loadConfigFile(configFilePath);

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const int* tris = m_geom->getMesh()->getTris();
	const int ntris = m_geom->getMesh()->getTriCount();

	//
	// Step 1. Initialize build config.
	//

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(m_cfg->bmin, bmin);
	rcVcopy(m_cfg->bmax, bmax);
	rcCalcGridSize(m_cfg->bmin, m_cfg->bmax, m_cfg->cs, &m_cfg->width, &m_cfg->height);

	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.	
	m_ctx->startTimer(RC_TIMER_TOTAL);

	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg->width, m_cfg->height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	rcHeightfield* m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg->width, m_cfg->height, m_cfg->bmin, m_cfg->bmax, m_cfg->cs, m_cfg->ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	unsigned char* m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(m_triareas, 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(m_ctx, m_cfg->walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg->walkableClimb))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	if (!m_keepInterResults)
	{
		delete[] m_triareas;
		m_triareas = 0;
	}

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg->walkableClimb, *m_solid);
	rcFilterLedgeSpans(m_ctx, m_cfg->walkableHeight, m_cfg->walkableClimb, *m_solid);
	rcFilterWalkableLowHeightSpans(m_ctx, m_cfg->walkableHeight, *m_solid);


	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	rcCompactHeightfield* m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg->walkableHeight, m_cfg->walkableClimb, *m_solid, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_cfg->walkableRadius, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);


	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg->minRegionArea, m_cfg->mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg->minRegionArea, m_cfg->mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg->minRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	rcContourSet* m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg->maxSimplificationError, m_cfg->maxEdgeLen, *m_cset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	rcPolyMesh* m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg->maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	rcPolyMeshDetail* m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg->detailSampleDist, m_cfg->detailSampleMaxError, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (m_cfg->maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg->cs;
		params.ch = m_cfg->ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		FILE* outputFile;
		if ((outputFile = fopen(outputFilePath.c_str(), "wb")) == NULL)
		{
			m_ctx->log(RC_LOG_ERROR, "Could not output File %s", outputFilePath);
			return false;
		}
		fwrite(&navData, navDataSize, 1, outputFile);
		fclose(outputFile);
		return true;
	}
	return 0;
};

int main(int argc, char** argv)
{
	if (argc != 0)
	{
		int i = 0;
		int m_partitionType = atoi(argv[i++]);
		float m_agentHeight = atof(argv[i++]);
		float m_agentRadius = atof(argv[i++]);
		float m_agentMaxClimb = atof(argv[i++]);
		const std::string indexsFilePath = argv[i++];
		const std::string vertexsFilePath = argv[i++];
		const std::string configFilePath = argv[i++];
		const std::string outputFilePath = argv[i++];
		buildFromOutSise(m_partitionType, m_agentHeight, m_agentRadius, m_agentMaxClimb, indexsFilePath, vertexsFilePath, configFilePath, outputFilePath);
		return 0;
	}
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL.\nError: %s\n", SDL_GetError());
		return -1;
	}

	// Enable depth buffer.
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	
	// Set color channel depth.
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	
	// 4x MSAA.
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	SDL_DisplayMode displayMode;
	SDL_GetCurrentDisplayMode(0, &displayMode);

	bool presentationMode = false;
	Uint32 flags = SDL_WINDOW_OPENGL;
	int width;
	int height;
	if (presentationMode)
	{
		// Create a fullscreen window at the native resolution.
		width = displayMode.w;
		height = displayMode.h;
		flags |= SDL_WINDOW_FULLSCREEN;
	}
	else
	{
		float aspect = 16.0f / 9.0f;
		width = rcMin(displayMode.w, (int)(displayMode.h * aspect)) - 80;
		height = displayMode.h - 80;
	}
	
	SDL_Window* window;
	SDL_Renderer* renderer;
	int errorCode = SDL_CreateWindowAndRenderer(width, height, flags, &window, &renderer);

	if (errorCode != 0 || !window || !renderer)
	{
		printf("Could not initialise SDL opengl\nError: %s\n", SDL_GetError());
		return -1;
	}

	SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
	SDL_GL_CreateContext(window);

	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		printf("Could not init GUI renderer.\n");
		SDL_Quit();
		return -1;
	}
	
	float t = 0.0f;
	float timeAcc = 0.0f;
	Uint32 prevFrameTime = SDL_GetTicks();
	int mousePos[2] = {0, 0};
	int origMousePos[2] = {0, 0}; // Used to compute mouse movement totals across frames.
	
	float cameraEulers[] = {45, -45};
	float cameraPos[] = {0, 0, 0};
	float camr = 1000;
	float origCameraEulers[] = {0, 0}; // Used to compute rotational changes across frames.
	
	float moveFront = 0.0f, moveBack = 0.0f, moveLeft = 0.0f, moveRight = 0.0f, moveUp = 0.0f, moveDown = 0.0f;
	
	float scrollZoom = 0;
	bool rotate = false;
	bool movedDuringRotate = false;
	float rayStart[3];
	float rayEnd[3];
	bool mouseOverMenu = false;
	
	bool showMenu = !presentationMode;
	bool showLog = false;
	bool showTools = true;
	bool showLevels = false;
	bool showSample = false;
	bool showTestCases = false;

	// Window scroll positions.
	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;
	
	string sampleName = "Choose Sample...";
	
	vector<string> files;
	const string meshesFolder = "Meshes";
	string meshName = "Choose Mesh...";
	
	float markerPosition[3] = {0, 0, 0};
	bool markerPositionSet = false;
	
	InputGeom* geom = 0;
	Sample* sample = 0;

	const string testCasesFolder = "TestCases";
	TestCase* test = 0;

	BuildContext ctx;
	
	// Fog.
	float fogColor[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);
	
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	
	bool done = false;
	while(!done)
	{
		// Handle input events.
		int mouseScroll = 0;
		bool processHitTest = false;
		bool processHitTestShift = false;
		SDL_Event event;
		
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_KEYDOWN:
					// Handle any key presses here.
					if (event.key.keysym.sym == SDLK_ESCAPE)
					{
						done = true;
					}
					else if (event.key.keysym.sym == SDLK_t)
					{
						showLevels = false;
						showSample = false;
						showTestCases = true;
						scanDirectory(testCasesFolder, ".txt", files);
					}
					else if (event.key.keysym.sym == SDLK_TAB)
					{
						showMenu = !showMenu;
					}
					else if (event.key.keysym.sym == SDLK_SPACE)
					{
						if (sample)
							sample->handleToggle();
					}
					else if (event.key.keysym.sym == SDLK_1)
					{
						if (sample)
							sample->handleStep();
					}
					else if (event.key.keysym.sym == SDLK_9)
					{
						if (sample && geom)
						{
							string savePath = meshesFolder + "/";
							BuildSettings settings;
							memset(&settings, 0, sizeof(settings));

							rcVcopy(settings.navMeshBMin, geom->getNavMeshBoundsMin());
							rcVcopy(settings.navMeshBMax, geom->getNavMeshBoundsMax());

							sample->collectSettings(settings);

							geom->saveGeomSet(&settings);
						}
					}
					break;
				
				case SDL_MOUSEWHEEL:
					if (event.wheel.y < 0)
					{
						// wheel down
						if (mouseOverMenu)
						{
							mouseScroll++;
						}
						else
						{
							scrollZoom += 1.0f;
						}
					}
					else
					{
						if (mouseOverMenu)
						{
							mouseScroll--;
						}
						else
						{
							scrollZoom -= 1.0f;
						}
					}
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_RIGHT)
					{
						if (!mouseOverMenu)
						{
							// Rotate view
							rotate = true;
							movedDuringRotate = false;
							origMousePos[0] = mousePos[0];
							origMousePos[1] = mousePos[1];
							origCameraEulers[0] = cameraEulers[0];
							origCameraEulers[1] = cameraEulers[1];
						}
					}
					break;
					
				case SDL_MOUSEBUTTONUP:
					// Handle mouse clicks here.
					if (event.button.button == SDL_BUTTON_RIGHT)
					{
						rotate = false;
						if (!mouseOverMenu)
						{
							if (!movedDuringRotate)
							{
								processHitTest = true;
								processHitTestShift = true;
							}
						}
					}
					else if (event.button.button == SDL_BUTTON_LEFT)
					{
						if (!mouseOverMenu)
						{
							processHitTest = true;
							processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
						}
					}
					
					break;
					
				case SDL_MOUSEMOTION:
					mousePos[0] = event.motion.x;
					mousePos[1] = height-1 - event.motion.y;
					
					if (rotate)
					{
						int dx = mousePos[0] - origMousePos[0];
						int dy = mousePos[1] - origMousePos[1];
						cameraEulers[0] = origCameraEulers[0] - dy * 0.25f;
						cameraEulers[1] = origCameraEulers[1] + dx * 0.25f;
						if (dx * dx + dy * dy > 3 * 3)
						{
							movedDuringRotate = true;
						}
					}
					break;
					
				case SDL_QUIT:
					done = true;
					break;
					
				default:
					break;
			}
		}

		unsigned char mouseButtonMask = 0;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
			mouseButtonMask |= IMGUI_MBUT_LEFT;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
			mouseButtonMask |= IMGUI_MBUT_RIGHT;
		
		Uint32 time = SDL_GetTicks();
		float dt = (time - prevFrameTime) / 1000.0f;
		prevFrameTime = time;
		
		t += dt;

		// Hit test mesh.
		if (processHitTest && geom && sample)
		{
			float hitTime;
			bool hit = geom->raycastMesh(rayStart, rayEnd, hitTime);
			
			if (hit)
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = true;
					markerPosition[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
					markerPosition[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
					markerPosition[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
				}
				else
				{
					float pos[3];
					pos[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
					pos[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
					pos[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
					sample->handleClick(rayStart, pos, processHitTestShift);
				}
			}
			else
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = false;
				}
			}
		}
		
		// Update sample simulation.
		const float SIM_RATE = 20;
		const float DELTA_TIME = 1.0f / SIM_RATE;
		timeAcc = rcClamp(timeAcc + dt, -1.0f, 1.0f);
		int simIter = 0;
		while (timeAcc > DELTA_TIME)
		{
			timeAcc -= DELTA_TIME;
			if (simIter < 5 && sample)
			{
				sample->handleUpdate(DELTA_TIME);
			}
			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		const float MIN_FRAME_TIME = 1.0f / 40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = (int)((MIN_FRAME_TIME - dt) * 1000.0f);
			if (ms > 10) ms = 10;
			if (ms >= 0) SDL_Delay(ms);
		}
		
		// Set the viewport.
		glViewport(0, 0, width, height);
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		
		// Clear the screen
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);
		
		// Compute the projection matrix.
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
		GLdouble projectionMatrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
		
		// Compute the modelview matrix.
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(cameraEulers[0], 1, 0, 0);
		glRotatef(cameraEulers[1], 0, 1, 0);
		glTranslatef(-cameraPos[0], -cameraPos[1], -cameraPos[2]);
		GLdouble modelviewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
		
		// Get hit ray position and direction.
		GLdouble x, y, z;
		gluUnProject(mousePos[0], mousePos[1], 0.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rayStart[0] = (float)x;
		rayStart[1] = (float)y;
		rayStart[2] = (float)z;
		gluUnProject(mousePos[0], mousePos[1], 1.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rayEnd[0] = (float)x;
		rayEnd[1] = (float)y;
		rayEnd[2] = (float)z;
		
		// Handle keyboard movement.
		const Uint8* keystate = SDL_GetKeyboardState(NULL);
		moveFront	= rcClamp(moveFront	+ dt * 4 * ((keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP		]) ? 1 : -1), 0.0f, 1.0f);
		moveLeft	= rcClamp(moveLeft	+ dt * 4 * ((keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_LEFT		]) ? 1 : -1), 0.0f, 1.0f);
		moveBack	= rcClamp(moveBack	+ dt * 4 * ((keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN		]) ? 1 : -1), 0.0f, 1.0f);
		moveRight	= rcClamp(moveRight	+ dt * 4 * ((keystate[SDL_SCANCODE_D] || keystate[SDL_SCANCODE_RIGHT	]) ? 1 : -1), 0.0f, 1.0f);
		moveUp		= rcClamp(moveUp	+ dt * 4 * ((keystate[SDL_SCANCODE_Q] || keystate[SDL_SCANCODE_PAGEUP	]) ? 1 : -1), 0.0f, 1.0f);
		moveDown	= rcClamp(moveDown	+ dt * 4 * ((keystate[SDL_SCANCODE_E] || keystate[SDL_SCANCODE_PAGEDOWN	]) ? 1 : -1), 0.0f, 1.0f);
		
		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
		{
			keybSpeed *= 4.0f;
		}
		
		float movex = (moveRight - moveLeft) * keybSpeed * dt;
		float movey = (moveBack - moveFront) * keybSpeed * dt + scrollZoom * 2.0f;
		scrollZoom = 0;
		
		cameraPos[0] += movex * (float)modelviewMatrix[0];
		cameraPos[1] += movex * (float)modelviewMatrix[4];
		cameraPos[2] += movex * (float)modelviewMatrix[8];
		
		cameraPos[0] += movey * (float)modelviewMatrix[2];
		cameraPos[1] += movey * (float)modelviewMatrix[6];
		cameraPos[2] += movey * (float)modelviewMatrix[10];

		cameraPos[1] += (moveUp - moveDown) * keybSpeed * dt;

		glEnable(GL_FOG);

		if (sample)
			sample->handleRender();
		if (test)
			test->handleRender();
		
		glDisable(GL_FOG);
		
		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		mouseOverMenu = false;
		
		imguiBeginFrame(mousePos[0], mousePos[1], mouseButtonMask, mouseScroll);
		
		if (sample)
		{
			sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);
		}
		if (test)
		{
			if (test->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
				mouseOverMenu = true;
		}

		// Help text.
		if (showMenu)
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate";
			imguiDrawText(280, height-20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255,255,255,128));
		}
		
		if (showMenu)
		{
			if (imguiBeginScrollArea("Properties", width-250-10, 10, 250, height-20, &propScroll))
				mouseOverMenu = true;

			if (imguiCheck("Show Log", showLog))
				showLog = !showLog;
			if (imguiCheck("Show Tools", showTools))
				showTools = !showTools;

			imguiSeparator();
			imguiLabel("Sample");
			if (imguiButton(sampleName.c_str()))
			{
				if (showSample)
				{
					showSample = false;
				}
				else
				{
					showSample = true;
					showLevels = false;
					showTestCases = false;
				}
			}
			
			imguiSeparator();
			imguiLabel("Input Mesh");
			if (imguiButton(meshName.c_str()))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				else
				{
					showSample = false;
					showTestCases = false;
					showLevels = true;
					scanDirectory(meshesFolder, ".obj", files);
					scanDirectoryAppend(meshesFolder, ".gset", files);
				}
			}
			if (geom)
			{
				char text[64];
				snprintf(text, 64, "Verts: %.1fk  Tris: %.1fk",
						 geom->getMesh()->getVertCount()/1000.0f,
						 geom->getMesh()->getTriCount()/1000.0f);
				imguiValue(text);
			}
			imguiSeparator();

			if (geom && sample)
			{
				imguiSeparatorLine();
				
				sample->handleSettings();

				if (imguiButton("Build"))
				{
					ctx.resetLog();
					if (!sample->handleBuild())
					{
						showLog = true;
						logScroll = 0;
					}
					ctx.dumpLog("Build log %s:", meshName.c_str());
					
					// Clear test.
					delete test;
					test = 0;
				}

				imguiSeparator();
			}
			
			if (sample)
			{
				imguiSeparatorLine();
				sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}
		
		// Sample selection dialog.
		if (showSample)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Sample", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))
				mouseOverMenu = true;

			Sample* newSample = 0;
			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name.c_str()))
				{
					newSample = g_samples[i].create();
					if (newSample)
						sampleName = g_samples[i].name;
				}
			}
			if (newSample)
			{
				delete sample;
				sample = newSample;
				sample->setContext(&ctx);
				if (geom)
				{
					sample->handleMeshChanged(geom);
				}
				showSample = false;
			}

			if (geom || sample)
			{
				const float* bmin = 0;
				const float* bmax = 0;
				if (geom)
				{
					bmin = geom->getNavMeshBoundsMin();
					bmax = geom->getNavMeshBoundsMax();
				}
				// Reset camera and fog to match the mesh bounds.
				if (bmin && bmax)
				{
					camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
								 rcSqr(bmax[1]-bmin[1]) +
								 rcSqr(bmax[2]-bmin[2])) / 2;
					cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
					cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
					cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
					camr *= 3;
				}
				cameraEulers[0] = 45;
				cameraEulers[1] = -45;
				glFogf(GL_FOG_START, camr*0.1f);
				glFogf(GL_FOG_END, camr*1.25f);
			}
			
			imguiEndScrollArea();
		}
		
		// Level selection dialog.
		if (showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Level", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &levelScroll))
				mouseOverMenu = true;
			
			vector<string>::const_iterator fileIter = files.begin();
			vector<string>::const_iterator filesEnd = files.end();
			vector<string>::const_iterator levelToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					levelToLoad = fileIter;
				}
			}
			
			if (levelToLoad != filesEnd)
			{
				meshName = *levelToLoad;
				showLevels = false;
				
				delete geom;
				geom = 0;
				
				string path = meshesFolder + "/" + meshName;
				
				geom = new InputGeom;
				if (!geom->load(&ctx, path))
				{
					delete geom;
					geom = 0;

					// Destroy the sample if it already had geometry loaded, as we've just deleted it!
					if (sample && sample->getInputGeom())
					{
						delete sample;
						sample = 0;
					}
					
					showLog = true;
					logScroll = 0;
					ctx.dumpLog("Geom load log %s:", meshName.c_str());
				}
				if (sample && geom)
				{
					sample->handleMeshChanged(geom);
				}

				if (geom || sample)
				{
					const float* bmin = 0;
					const float* bmax = 0;
					if (geom)
					{
						bmin = geom->getNavMeshBoundsMin();
						bmax = geom->getNavMeshBoundsMax();
					}
					// Reset camera and fog to match the mesh bounds.
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
									 rcSqr(bmax[1]-bmin[1]) +
									 rcSqr(bmax[2]-bmin[2])) / 2;
						cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
						cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
						cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}
					cameraEulers[0] = 45;
					cameraEulers[1] = -45;
					glFogf(GL_FOG_START, camr * 0.1f);
					glFogf(GL_FOG_END, camr * 1.25f);
				}
			}
			
			imguiEndScrollArea();
			
		}
		
		// Test cases
		if (showTestCases)
		{
			static int testScroll = 0;
			if (imguiBeginScrollArea("Choose Test To Run", width-10-250-10-200, height-10-450, 200, 450, &testScroll))
				mouseOverMenu = true;

			vector<string>::const_iterator fileIter = files.begin();
			vector<string>::const_iterator filesEnd = files.end();
			vector<string>::const_iterator testToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					testToLoad = fileIter;
				}
			}
			
			if (testToLoad != filesEnd)
			{
				string path = testCasesFolder + "/" + *testToLoad;
				test = new TestCase;
				if (test)
				{
					// Load the test.
					if (!test->load(path))
					{
						delete test;
						test = 0;
					}

					// Create sample
					Sample* newSample = 0;
					for (int i = 0; i < g_nsamples; ++i)
					{
						if (g_samples[i].name == test->getSampleName())
						{
							newSample = g_samples[i].create();
							if (newSample)
								sampleName = g_samples[i].name;
						}
					}

					delete sample;
					sample = newSample;

					if (sample)
					{
						sample->setContext(&ctx);
						showSample = false;
					}

					// Load geom.
					meshName = test->getGeomFileName();
					
					
					path = meshesFolder + "/" + meshName;
					
					delete geom;
					geom = new InputGeom;
					if (!geom || !geom->load(&ctx, path))
					{
						delete geom;
						geom = 0;
						delete sample;
						sample = 0;
						showLog = true;
						logScroll = 0;
						ctx.dumpLog("Geom load log %s:", meshName.c_str());
					}
					if (sample && geom)
					{
						sample->handleMeshChanged(geom);
					}

					// This will ensure that tile & poly bits are updated in tiled sample.
					if (sample)
						sample->handleSettings();

					ctx.resetLog();
					if (sample && !sample->handleBuild())
					{
						ctx.dumpLog("Build log %s:", meshName.c_str());
					}
					
					if (geom || sample)
					{
						const float* bmin = 0;
						const float* bmax = 0;
						if (geom)
						{
							bmin = geom->getNavMeshBoundsMin();
							bmax = geom->getNavMeshBoundsMax();
						}
						// Reset camera and fog to match the mesh bounds.
						if (bmin && bmax)
						{
							camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
										 rcSqr(bmax[1] - bmin[1]) +
										 rcSqr(bmax[2] - bmin[2])) / 2;
							cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
							cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
							cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
							camr *= 3;
						}
						cameraEulers[0] = 45;
						cameraEulers[1] = -45;
						glFogf(GL_FOG_START, camr * 0.2f);
						glFogf(GL_FOG_END, camr * 1.25f);
					}
					
					// Do the tests.
					//if (sample)
					//	test->doTests(sample->getNavMesh(), sample->getNavMeshQuery());
				}
			}				
				
			imguiEndScrollArea();
		}

		
		// Log
		if (showLog && showMenu)
		{
			if (imguiBeginScrollArea("Log", 250 + 20, 10, width - 300 - 250, 200, &logScroll))
				mouseOverMenu = true;
			for (int i = 0; i < ctx.getLogCount(); ++i)
				imguiLabel(ctx.getLogText(i));
			imguiEndScrollArea();
		}
		
		// Left column tools menu
		if (!showTestCases && showTools && showMenu) // && geom && sample)
		{
			if (imguiBeginScrollArea("Tools", 10, 10, 250, height - 20, &toolsScroll))
				mouseOverMenu = true;

			if (sample)
				sample->handleTools();
			
			imguiEndScrollArea();
		}
		
		// Marker
		if (markerPositionSet && gluProject((GLdouble)markerPosition[0], (GLdouble)markerPosition[1], (GLdouble)markerPosition[2],
								  modelviewMatrix, projectionMatrix, viewport, &x, &y, &z))
		{
			// Draw marker circle
			glLineWidth(5.0f);
			glColor4ub(240,220,0,196);
			glBegin(GL_LINE_LOOP);
			const float r = 25.0f;
			for (int i = 0; i < 20; ++i)
			{
				const float a = (float)i / 20.0f * RC_PI*2;
				const float fx = (float)x + cosf(a)*r;
				const float fy = (float)y + sinf(a)*r;
				glVertex2f(fx,fy);
			}
			glEnd();
			glLineWidth(1.0f);
		}
		
		imguiEndFrame();
		imguiRenderGLDraw();		
		
		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapWindow(window);
	}
	
	imguiRenderGLDestroy();
	
	SDL_Quit();
	
	delete sample;
	delete geom;
	
	return 0;
}
