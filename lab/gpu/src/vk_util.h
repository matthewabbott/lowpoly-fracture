// vk_util.h: a minimal Vulkan compute layer (instance, devices, buffers, pipelines, timestamps).
#ifndef VK_UTIL_H
#define VK_UTIL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <vulkan/vulkan.h>

#define VK_CHECK( x )                                                                                                  \
	do                                                                                                                 \
	{                                                                                                                  \
		VkResult r_ = ( x );                                                                                           \
		if ( r_ != VK_SUCCESS )                                                                                        \
		{                                                                                                              \
			fprintf( stderr, "Vulkan error %d at %s:%d: %s\n", (int)r_, __FILE__, __LINE__, #x );                   \
			exit( 1 );                                                                                                 \
		}                                                                                                              \
	} while ( 0 )

typedef struct VkBuf
{
	VkBuffer buffer;
	VkDeviceMemory memory;
	VkDeviceSize size;
	void* mapped; // non-null for host-visible buffers
} VkBuf;

typedef struct VkGpu
{
	VkPhysicalDevice phys;
	VkDevice device;
	VkQueue queue;
	uint32_t queueFamily;
	VkPhysicalDeviceProperties props;
	VkPhysicalDeviceFloatControlsProperties floatControls;
	VkPhysicalDeviceMemoryProperties memProps;
	float timestampPeriod;
	uint32_t timestampValidBits;
	bool execProps; // VK_KHR_pipeline_executable_properties enabled
	VkCommandPool cmdPool;
	char vendor[16]; // "nvidia", "intel", "amd", "apple", "arm", "llvmpipe", "mesa", "other"
} VkGpu;

VkInstance vku_create_instance( void );
int vku_list_gpus( VkInstance inst, VkPhysicalDevice* out, int max );
void vku_open_gpu( VkInstance inst, VkPhysicalDevice phys, VkGpu* g );
void vku_close_gpu( VkGpu* g );

VkBuf vku_buffer( VkGpu* g, VkDeviceSize size, bool hostVisible );
void vku_free( VkGpu* g, VkBuf* b );

// Compute pipeline from a SPIR-V file; `layout` is shared. Returns VK_NULL_HANDLE on a missing file.
VkPipeline vku_pipeline( VkGpu* g, VkPipelineLayout layout, const char* spvPath, const char* entry );
// Prints the driver's statistics (registers, instructions, ...) for a pipeline, when available.
void vku_print_pipeline_stats( VkGpu* g, VkPipeline pipe, const char* label, FILE* out );

VkCommandBuffer vku_begin( VkGpu* g );
void vku_submit_wait( VkGpu* g, VkCommandBuffer cb ); // ends, submits, waits, frees
void vku_barrier( VkCommandBuffer cb );				   // compute+transfer write -> compute+transfer read/write

double vku_now_ms( void );

// Whether the device advertises every float-control execution mode a SPIR-V file tag names (gen.py's
// suffixes: "", ".rte", ".szinp", ".preserve", ".ftz", ".rtesz", ".pszinp"). A mode the device does not
// advertise is undefined behaviour, so the runners skip it; `missing` names what is absent.
bool vku_tag_supported( const VkGpu* g, const char* tag, char* missing, size_t missingSize );

#endif
