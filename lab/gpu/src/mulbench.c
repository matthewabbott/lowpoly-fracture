// mulbench.c: runs mulbench.comp (modes 0-7) on every GPU and prints multiply throughput.
#include "vk_util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* g_modes[8] = { "fp32 fma", "int32 mul-add", "i32*i32->i64 (sext) >>20", "int64*int64 (full)", "imulExtended hi/lo",
								  "umulExtended hi/lo", "Q32.32 qmul (limbs)", "i32*i32->i64 round+var shift" };

int main( int argc, char** argv )
{
	const char* logPath = argc > 1 ? argv[1] : "logs/mulbench.txt";
	FILE* log = fopen( logPath, "w" );
	if ( !log )
		return 1;
	uint32_t threads = 1u << 20;
	VkInstance inst = vku_create_instance();
	VkPhysicalDevice phys[8];
	int n = vku_list_gpus( inst, phys, 8 );
	for ( int gi = 0; gi < n; ++gi )
	{
		VkGpu g;
		vku_open_gpu( inst, phys[gi], &g );
		fprintf( log, "\n%s (%s): %u threads x 4 chains x 256 steps per dispatch, median of 5\n", g.props.deviceName, g.vendor, threads );
		VkDescriptorSetLayoutBinding b = { 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
		VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
		dci.bindingCount = 1;
		dci.pBindings = &b;
		VkDescriptorSetLayout dsl;
		VK_CHECK( vkCreateDescriptorSetLayout( g.device, &dci, NULL, &dsl ) );
		VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, 16 };
		VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
		lci.setLayoutCount = 1;
		lci.pSetLayouts = &dsl;
		lci.pushConstantRangeCount = 1;
		lci.pPushConstantRanges = &pcr;
		VkPipelineLayout layout;
		VK_CHECK( vkCreatePipelineLayout( g.device, &lci, NULL, &layout ) );
		VkBuf buf = vku_buffer( &g, (VkDeviceSize)threads * 8, false );
		VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1 };
		VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
		pci.maxSets = 1;
		pci.poolSizeCount = 1;
		pci.pPoolSizes = &ps;
		VkDescriptorPool pool;
		VK_CHECK( vkCreateDescriptorPool( g.device, &pci, NULL, &pool ) );
		VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
		ai.descriptorPool = pool;
		ai.descriptorSetCount = 1;
		ai.pSetLayouts = &dsl;
		VkDescriptorSet set;
		VK_CHECK( vkAllocateDescriptorSets( g.device, &ai, &set ) );
		VkDescriptorBufferInfo bi = { buf.buffer, 0, VK_WHOLE_SIZE };
		VkWriteDescriptorSet w = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
		w.dstSet = set;
		w.dstBinding = 0;
		w.descriptorCount = 1;
		w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		w.pBufferInfo = &bi;
		vkUpdateDescriptorSets( g.device, 1, &w, 0, NULL );
		VkQueryPoolCreateInfo qci = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
		qci.queryType = VK_QUERY_TYPE_TIMESTAMP;
		qci.queryCount = 2;
		VkQueryPool qp;
		VK_CHECK( vkCreateQueryPool( g.device, &qci, NULL, &qp ) );
		double base = 0.0;
		for ( int m = 0; m < 8; ++m )
		{
			char path[128];
			snprintf( path, sizeof( path ), "gen/mulbench/mode%d.spv", m );
			VkPipeline pipe = vku_pipeline( &g, layout, path, "main" );
			if ( pipe == VK_NULL_HANDLE )
				continue;
			double t[6];
			for ( int rep = 0; rep < 6; ++rep )
			{
				VkCommandBuffer cb = vku_begin( &g );
				vkCmdResetQueryPool( cb, qp, 0, 2 );
				vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe );
				vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, layout, 0, 1, &set, 0, NULL );
				uint32_t push[4] = { (uint32_t)rep, 20, 0, 0 };
				vkCmdPushConstants( cb, layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, 16, push );
				vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, qp, 0 );
				vkCmdDispatch( cb, threads / 64, 1, 1 );
				vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, qp, 1 );
				vku_submit_wait( &g, cb );
				uint64_t ts[2];
				VK_CHECK( vkGetQueryPoolResults( g.device, qp, 0, 2, sizeof( ts ), ts, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
				t[rep] = (double)( ts[1] - ts[0] ) * (double)g.timestampPeriod * 1e-9;
			}
			// median of reps 1..5
			for ( int a = 1; a < 6; ++a )
				for ( int c = a + 1; c < 6; ++c )
					if ( t[c] < t[a] )
					{
						double x = t[a];
						t[a] = t[c];
						t[c] = x;
					}
			double sec = t[3];
			double gops = (double)threads * 4.0 * 256.0 / sec * 1e-9;
			if ( m == 0 )
				base = gops;
			fprintf( log, "  %-32s %8.2f G steps/s  (%.2fx the fp32 fma rate, %.3f ms)\n", g_modes[m], gops, base / gops, sec * 1e3 );
			vkDestroyPipeline( g.device, pipe, NULL );
		}
		vkDestroyQueryPool( g.device, qp, NULL );
		vkDestroyDescriptorPool( g.device, pool, NULL );
		vku_free( &g, &buf );
		vkDestroyPipelineLayout( g.device, layout, NULL );
		vkDestroyDescriptorSetLayout( g.device, dsl, NULL );
		vku_close_gpu( &g );
	}
	vkDestroyInstance( inst, NULL );
	fclose( log );
	printf( "done: %s\n", logPath );
	return 0;
}
