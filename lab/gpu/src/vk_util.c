// vk_util.c: see vk_util.h.
#include "vk_util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

double vku_now_ms( void )
{
	static LARGE_INTEGER freq;
	if ( freq.QuadPart == 0 )
	{
		QueryPerformanceFrequency( &freq );
	}
	LARGE_INTEGER t;
	QueryPerformanceCounter( &t );
	return 1000.0 * (double)t.QuadPart / (double)freq.QuadPart;
}
#else
#include <time.h>

double vku_now_ms( void )
{
	struct timespec t;
	clock_gettime( CLOCK_MONOTONIC, &t );
	return 1000.0 * (double)t.tv_sec + 1e-6 * (double)t.tv_nsec;
}
#endif

static bool has_instance_extension( const char* name )
{
	uint32_t n = 0;
	vkEnumerateInstanceExtensionProperties( NULL, &n, NULL );
	VkExtensionProperties* e = (VkExtensionProperties*)malloc( ( n ? n : 1 ) * sizeof( *e ) );
	vkEnumerateInstanceExtensionProperties( NULL, &n, e );
	bool found = false;
	for ( uint32_t i = 0; i < n; ++i )
	{
		if ( strcmp( e[i].extensionName, name ) == 0 )
		{
			found = true;
		}
	}
	free( e );
	return found;
}

bool vku_tag_supported( const VkGpu* g, const char* tag, char* missing, size_t missingSize )
{
	const VkPhysicalDeviceFloatControlsProperties* fc = &g->floatControls;
	bool rte = strstr( tag, "rte" ) != NULL;
	bool sz = strstr( tag, "sz" ) != NULL;
	bool preserve = strcmp( tag, ".preserve" ) == 0 || strcmp( tag, ".pszinp" ) == 0;
	bool ftz = strcmp( tag, ".ftz" ) == 0;
	snprintf( missing, missingSize, "%s%s%s%s", rte && !fc->shaderRoundingModeRTEFloat32 ? " RoundingModeRTE" : "",
			  sz && !fc->shaderSignedZeroInfNanPreserveFloat32 ? " SignedZeroInfNanPreserve" : "",
			  preserve && !fc->shaderDenormPreserveFloat32 ? " DenormPreserve" : "",
			  ftz && !fc->shaderDenormFlushToZeroFloat32 ? " DenormFlushToZero" : "" );
	return missing[0] == 0;
}

VkInstance vku_create_instance( void )
{
	VkApplicationInfo app = { VK_STRUCTURE_TYPE_APPLICATION_INFO };
	app.pApplicationName = "m7-gpu-e11";
	app.apiVersion = VK_API_VERSION_1_3;
	VkInstanceCreateInfo ci = { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
	ci.pApplicationInfo = &app;
	// MoltenVK (macOS) is a portability driver: it is listed only when asked for
	const char* exts[1];
	if ( has_instance_extension( "VK_KHR_portability_enumeration" ) )
	{
		exts[0] = "VK_KHR_portability_enumeration";
		ci.enabledExtensionCount = 1;
		ci.ppEnabledExtensionNames = exts;
		ci.flags |= 0x00000001; // VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR
	}
	const char* layers[] = { "VK_LAYER_KHRONOS_validation" };
	if ( getenv( "E11_VALIDATION" ) != NULL )
	{
		ci.enabledLayerCount = 1;
		ci.ppEnabledLayerNames = layers;
	}
	VkInstance inst;
	VK_CHECK( vkCreateInstance( &ci, NULL, &inst ) );
	return inst;
}

int vku_list_gpus( VkInstance inst, VkPhysicalDevice* out, int max )
{
	uint32_t n = 0;
	VK_CHECK( vkEnumeratePhysicalDevices( inst, &n, NULL ) );
	if ( (int)n > max )
	{
		n = (uint32_t)max;
	}
	VK_CHECK( vkEnumeratePhysicalDevices( inst, &n, out ) );
	return (int)n;
}

static bool has_extension( VkPhysicalDevice phys, const char* name )
{
	uint32_t n = 0;
	vkEnumerateDeviceExtensionProperties( phys, NULL, &n, NULL );
	VkExtensionProperties* e = (VkExtensionProperties*)malloc( n * sizeof( *e ) );
	vkEnumerateDeviceExtensionProperties( phys, NULL, &n, e );
	bool found = false;
	for ( uint32_t i = 0; i < n; ++i )
	{
		if ( strcmp( e[i].extensionName, name ) == 0 )
		{
			found = true;
		}
	}
	free( e );
	return found;
}

void vku_open_gpu( VkInstance inst, VkPhysicalDevice phys, VkGpu* g )
{
	(void)inst;
	memset( g, 0, sizeof( *g ) );
	g->phys = phys;

	g->floatControls.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FLOAT_CONTROLS_PROPERTIES;
	VkPhysicalDeviceProperties2 p2 = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	p2.pNext = &g->floatControls;
	vkGetPhysicalDeviceProperties2( phys, &p2 );
	g->props = p2.properties;
	g->timestampPeriod = g->props.limits.timestampPeriod;
	vkGetPhysicalDeviceMemoryProperties( phys, &g->memProps );
	uint32_t vid = g->props.vendorID;
	const char* vname = vid == 0x10de ? "nvidia" : vid == 0x8086 ? "intel" : vid == 0x1002 ? "amd" : vid == 0x106b ? "apple" : vid == 0x13b5 ? "arm" : "other";
	if ( vid == 0x10005 ) // VK_VENDOR_ID_MESA: llvmpipe (software) or another Mesa driver
	{
		vname = strstr( g->props.deviceName, "llvmpipe" ) != NULL ? "llvmpipe" : "mesa";
	}
	snprintf( g->vendor, sizeof( g->vendor ), "%s", vname );

	uint32_t qn = 0;
	vkGetPhysicalDeviceQueueFamilyProperties( phys, &qn, NULL );
	VkQueueFamilyProperties* qf = (VkQueueFamilyProperties*)malloc( qn * sizeof( *qf ) );
	vkGetPhysicalDeviceQueueFamilyProperties( phys, &qn, qf );
	g->queueFamily = UINT32_MAX;
	for ( uint32_t i = 0; i < qn; ++i )
	{
		if ( ( qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT ) && qf[i].timestampValidBits > 0 )
		{
			g->queueFamily = i;
			g->timestampValidBits = qf[i].timestampValidBits;
			break;
		}
	}
	free( qf );
	if ( g->queueFamily == UINT32_MAX )
	{
		fprintf( stderr, "no compute queue with timestamps on %s\n", g->props.deviceName );
		exit( 1 );
	}

	float prio = 1.0f;
	VkDeviceQueueCreateInfo qci = { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
	qci.queueFamilyIndex = g->queueFamily;
	qci.queueCount = 1;
	qci.pQueuePriorities = &prio;

	VkPhysicalDevicePipelineExecutablePropertiesFeaturesKHR execF = {
		VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PIPELINE_EXECUTABLE_PROPERTIES_FEATURES_KHR };
	VkPhysicalDeviceFeatures2 f2 = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
	const char* exts[4];
	uint32_t extCount = 0;
	if ( has_extension( phys, "VK_KHR_portability_subset" ) ) // required when a portability driver offers it
	{
		exts[extCount++] = "VK_KHR_portability_subset";
	}
	g->execProps = has_extension( phys, VK_KHR_PIPELINE_EXECUTABLE_PROPERTIES_EXTENSION_NAME );
	if ( g->execProps )
	{
		exts[extCount++] = VK_KHR_PIPELINE_EXECUTABLE_PROPERTIES_EXTENSION_NAME;
		execF.pipelineExecutableInfo = VK_TRUE;
		f2.pNext = &execF;
	}
	f2.features.shaderInt64 = VK_TRUE;

	VkDeviceCreateInfo dci = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
	dci.pNext = &f2;
	dci.queueCreateInfoCount = 1;
	dci.pQueueCreateInfos = &qci;
	dci.enabledExtensionCount = extCount;
	dci.ppEnabledExtensionNames = exts;
	VK_CHECK( vkCreateDevice( phys, &dci, NULL, &g->device ) );
	vkGetDeviceQueue( g->device, g->queueFamily, 0, &g->queue );

	VkCommandPoolCreateInfo pci = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
	pci.queueFamilyIndex = g->queueFamily;
	pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	VK_CHECK( vkCreateCommandPool( g->device, &pci, NULL, &g->cmdPool ) );
}

void vku_close_gpu( VkGpu* g )
{
	vkDestroyCommandPool( g->device, g->cmdPool, NULL );
	vkDestroyDevice( g->device, NULL );
}

static uint32_t find_memory( VkGpu* g, uint32_t bits, VkMemoryPropertyFlags want, VkMemoryPropertyFlags avoid )
{
	for ( uint32_t i = 0; i < g->memProps.memoryTypeCount; ++i )
	{
		VkMemoryPropertyFlags f = g->memProps.memoryTypes[i].propertyFlags;
		if ( ( bits & ( 1u << i ) ) && ( f & want ) == want && ( f & avoid ) == 0 )
		{
			return i;
		}
	}
	return UINT32_MAX;
}

VkBuf vku_buffer( VkGpu* g, VkDeviceSize size, bool hostVisible )
{
	VkBuf b = { 0 };
	b.size = size;
	VkBufferCreateInfo bci = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
	bci.size = size;
	bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	VK_CHECK( vkCreateBuffer( g->device, &bci, NULL, &b.buffer ) );
	VkMemoryRequirements req;
	vkGetBufferMemoryRequirements( g->device, b.buffer, &req );
	uint32_t type;
	if ( hostVisible )
	{
		// prefer cached host memory for readback
		type = find_memory( g, req.memoryTypeBits,
							VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT, 0 );
		if ( type == UINT32_MAX )
		{
			type = find_memory( g, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 0 );
		}
	}
	else
	{
		type = find_memory( g, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0 );
	}
	if ( type == UINT32_MAX )
	{
		fprintf( stderr, "no memory type\n" );
		exit( 1 );
	}
	VkMemoryAllocateInfo mai = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	mai.allocationSize = req.size;
	mai.memoryTypeIndex = type;
	VK_CHECK( vkAllocateMemory( g->device, &mai, NULL, &b.memory ) );
	VK_CHECK( vkBindBufferMemory( g->device, b.buffer, b.memory, 0 ) );
	if ( hostVisible )
	{
		VK_CHECK( vkMapMemory( g->device, b.memory, 0, VK_WHOLE_SIZE, 0, &b.mapped ) );
	}
	return b;
}

void vku_free( VkGpu* g, VkBuf* b )
{
	if ( b->buffer == VK_NULL_HANDLE )
	{
		return;
	}
	if ( b->mapped )
	{
		vkUnmapMemory( g->device, b->memory );
	}
	vkDestroyBuffer( g->device, b->buffer, NULL );
	vkFreeMemory( g->device, b->memory, NULL );
	memset( b, 0, sizeof( *b ) );
}

VkPipeline vku_pipeline( VkGpu* g, VkPipelineLayout layout, const char* spvPath, const char* entry )
{
	FILE* f = fopen( spvPath, "rb" );
	if ( f == NULL )
	{
		fprintf( stderr, "missing %s\n", spvPath );
		return VK_NULL_HANDLE;
	}
	fseek( f, 0, SEEK_END );
	long n = ftell( f );
	fseek( f, 0, SEEK_SET );
	uint32_t* code = (uint32_t*)malloc( (size_t)n );
	fread( code, 1, (size_t)n, f );
	fclose( f );

	VkShaderModuleCreateInfo smci = { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
	smci.codeSize = (size_t)n;
	smci.pCode = code;
	VkShaderModule sm;
	VK_CHECK( vkCreateShaderModule( g->device, &smci, NULL, &sm ) );
	free( code );

	VkComputePipelineCreateInfo cpci = { VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
	cpci.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	cpci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	cpci.stage.module = sm;
	cpci.stage.pName = entry;
	cpci.layout = layout;
	const char* irDir = getenv( "E11_DUMP_IR" );
	if ( g->execProps )
	{
		cpci.flags |= VK_PIPELINE_CREATE_CAPTURE_STATISTICS_BIT_KHR;
		if ( irDir != NULL )
		{
			cpci.flags |= VK_PIPELINE_CREATE_CAPTURE_INTERNAL_REPRESENTATIONS_BIT_KHR;
		}
	}
	VkPipeline p;
	if ( getenv( "E11_TRACE_PIPELINES" ) != NULL ) // names the kernel when a driver's compiler crashes
	{
		fprintf( stderr, "pipeline %s on %s\n", spvPath, g->props.deviceName );
	}
	VK_CHECK( vkCreateComputePipelines( g->device, VK_NULL_HANDLE, 1, &cpci, NULL, &p ) );
	vkDestroyShaderModule( g->device, sm, NULL );
	if ( g->execProps && irDir != NULL )
	{
		PFN_vkGetPipelineExecutableInternalRepresentationsKHR getIR = (PFN_vkGetPipelineExecutableInternalRepresentationsKHR)vkGetDeviceProcAddr(
			g->device, "vkGetPipelineExecutableInternalRepresentationsKHR" );
		VkPipelineExecutableInfoKHR ei = { VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_INFO_KHR };
		ei.pipeline = p;
		ei.executableIndex = 0;
		uint32_t n = 0;
		VkResult irr = getIR != NULL ? getIR( g->device, &ei, &n, NULL ) : VK_ERROR_FEATURE_NOT_PRESENT;
		fprintf( stderr, "IR %s %s: result %d, %u representations\n", g->vendor, spvPath, (int)irr, n );
		if ( irr == VK_SUCCESS && n > 0 )
		{
			VkPipelineExecutableInternalRepresentationKHR* ir = (VkPipelineExecutableInternalRepresentationKHR*)calloc( n, sizeof( *ir ) );
			for ( uint32_t i = 0; i < n; ++i )
			{
				ir[i].sType = VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_INTERNAL_REPRESENTATION_KHR;
			}
			getIR( g->device, &ei, &n, ir );
			for ( uint32_t i = 0; i < n; ++i )
			{
				ir[i].pData = calloc( 1, ir[i].dataSize + 1 );
			}
			getIR( g->device, &ei, &n, ir );
			const char* base = strrchr( spvPath, '/' );
			base = base ? base + 1 : spvPath;
			for ( uint32_t i = 0; i < n; ++i )
			{
				char path[600];
				snprintf( path, sizeof( path ), "%s/%s_%s_ir%u.txt", irDir, g->vendor, base, i );
				FILE* f = fopen( path, "wb" );
				if ( f )
				{
					fprintf( f, "# %s: %s\n", ir[i].name, ir[i].description );
					if ( ir[i].isText )
					{
						fwrite( ir[i].pData, 1, strlen( (const char*)ir[i].pData ), f );
					}
					fclose( f );
				}
				free( ir[i].pData );
			}
			free( ir );
		}
	}
	return p;
}

void vku_print_pipeline_stats( VkGpu* g, VkPipeline pipe, const char* label, FILE* log )
{
	if ( !g->execProps )
	{
		return;
	}
	PFN_vkGetPipelineExecutablePropertiesKHR getProps =
		(PFN_vkGetPipelineExecutablePropertiesKHR)vkGetDeviceProcAddr( g->device, "vkGetPipelineExecutablePropertiesKHR" );
	PFN_vkGetPipelineExecutableStatisticsKHR getStats =
		(PFN_vkGetPipelineExecutableStatisticsKHR)vkGetDeviceProcAddr( g->device, "vkGetPipelineExecutableStatisticsKHR" );
	if ( getProps == NULL || getStats == NULL )
	{
		return;
	}
	VkPipelineInfoKHR pi = { VK_STRUCTURE_TYPE_PIPELINE_INFO_KHR };
	pi.pipeline = pipe;
	uint32_t n = 0;
	getProps( g->device, &pi, &n, NULL );
	if ( n > 8 )
	{
		n = 8;
	}
	VkPipelineExecutablePropertiesKHR ep[8];
	for ( uint32_t i = 0; i < 8; ++i )
	{
		ep[i].sType = VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_PROPERTIES_KHR;
		ep[i].pNext = NULL;
	}
	getProps( g->device, &pi, &n, ep );
	for ( uint32_t e = 0; e < n; ++e )
	{
		VkPipelineExecutableInfoKHR ei = { VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_INFO_KHR };
		ei.pipeline = pipe;
		ei.executableIndex = e;
		uint32_t sn = 0;
		getStats( g->device, &ei, &sn, NULL );
		VkPipelineExecutableStatisticKHR* st = (VkPipelineExecutableStatisticKHR*)calloc( sn ? sn : 1, sizeof( *st ) );
		for ( uint32_t i = 0; i < sn; ++i )
		{
			st[i].sType = VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_STATISTIC_KHR;
		}
		getStats( g->device, &ei, &sn, st );
		fprintf( log, "  stats %-10s %s [%s, subgroup %u]:", label, g->vendor, ep[e].name, ep[e].subgroupSize );
		for ( uint32_t i = 0; i < sn; ++i )
		{
			switch ( st[i].format )
			{
				case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_BOOL32_KHR:
					fprintf( log, " %s=%u;", st[i].name, st[i].value.b32 );
					break;
				case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_INT64_KHR:
					fprintf( log, " %s=%lld;", st[i].name, (long long)st[i].value.i64 );
					break;
				case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_UINT64_KHR:
					fprintf( log, " %s=%llu;", st[i].name, (unsigned long long)st[i].value.u64 );
					break;
				default:
					fprintf( log, " %s=%g;", st[i].name, st[i].value.f64 );
					break;
			}
		}
		fprintf( log, "\n" );
		free( st );
	}
}

VkCommandBuffer vku_begin( VkGpu* g )
{
	VkCommandBufferAllocateInfo ai = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	ai.commandPool = g->cmdPool;
	ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	ai.commandBufferCount = 1;
	VkCommandBuffer cb;
	VK_CHECK( vkAllocateCommandBuffers( g->device, &ai, &cb ) );
	VkCommandBufferBeginInfo bi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK( vkBeginCommandBuffer( cb, &bi ) );
	return cb;
}

void vku_submit_wait( VkGpu* g, VkCommandBuffer cb )
{
	VK_CHECK( vkEndCommandBuffer( cb ) );
	VkFenceCreateInfo fci = { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
	VkFence fence;
	VK_CHECK( vkCreateFence( g->device, &fci, NULL, &fence ) );
	VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
	si.commandBufferCount = 1;
	si.pCommandBuffers = &cb;
	VK_CHECK( vkQueueSubmit( g->queue, 1, &si, fence ) );
	VK_CHECK( vkWaitForFences( g->device, 1, &fence, VK_TRUE, UINT64_MAX ) );
	vkDestroyFence( g->device, fence, NULL );
	vkFreeCommandBuffers( g->device, g->cmdPool, 1, &cb );
}

void vku_barrier( VkCommandBuffer cb )
{
	VkMemoryBarrier mb = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
	mb.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
						  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &mb, 0, NULL, 0, NULL );
}
