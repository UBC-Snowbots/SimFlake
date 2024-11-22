using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class GroundTruthRendererFeature : ScriptableRendererFeature
{
    class GroundTruthRenderPass : ScriptableRenderPass
    {
        private RTHandle cameraColorTargetHandle;
        private string profilerTag;
        private Material groundTruthMaterial;

        public GroundTruthRenderPass(string profilerTag, Material groundTruthMaterial)
        {
            this.profilerTag = profilerTag;
            this.groundTruthMaterial = groundTruthMaterial;
        }

        public void Setup(RTHandle cameraColorTargetHandle)
        {
            this.cameraColorTargetHandle = cameraColorTargetHandle;
        }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
        {
            if (groundTruthMaterial == null) return;

            CommandBuffer cmd = CommandBufferPool.Get(profilerTag);
            using (new ProfilingScope(cmd, new ProfilingSampler(profilerTag)))
            {
                // Use the updated Blit method with RTHandles
                Blit(cmd, cameraColorTargetHandle, cameraColorTargetHandle, groundTruthMaterial);
            }
            context.ExecuteCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
        }
    }

    [SerializeField] private Material groundTruthMaterial;
    private GroundTruthRenderPass groundTruthRenderPass;

    public override void Create()
    {
        groundTruthRenderPass = new GroundTruthRenderPass("GroundTruth Render Pass", groundTruthMaterial)
        {
            renderPassEvent = RenderPassEvent.AfterRenderingOpaques
        };
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        if (groundTruthMaterial != null)
        {
            groundTruthRenderPass.Setup(renderer.cameraColorTargetHandle);
            renderer.EnqueuePass(groundTruthRenderPass);
        }
    }
}

