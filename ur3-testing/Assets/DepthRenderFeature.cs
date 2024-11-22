using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class DepthRenderFeature : ScriptableRendererFeature
{
    class DepthRenderPass : ScriptableRenderPass
    {
        private RenderTargetHandle depthTextureHandle;
        private ShaderTagId shaderTagId = new ShaderTagId("DepthOnly");
        private FilteringSettings filteringSettings = new FilteringSettings(RenderQueueRange.opaque);

        public DepthRenderPass()
        {
            depthTextureHandle.Init("_CameraDepthTexture");
        }

        public override void Configure(CommandBuffer cmd, RenderTextureDescriptor cameraTextureDescriptor)
        {
            cmd.GetTemporaryRT(depthTextureHandle.id, cameraTextureDescriptor);
            ConfigureTarget(depthTextureHandle.Identifier());
            ConfigureClear(ClearFlag.All, Color.black);
        }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
        {
            var drawingSettings = CreateDrawingSettings(shaderTagId, ref renderingData, SortingCriteria.CommonOpaque);
            context.DrawRenderers(renderingData.cullResults, ref drawingSettings, ref filteringSettings);
        }

        public override void FrameCleanup(CommandBuffer cmd)
        {
            cmd.ReleaseTemporaryRT(depthTextureHandle.id);
        }
    }

    DepthRenderPass depthRenderPass;

    public override void Create()
    {
        depthRenderPass = new DepthRenderPass
        {
            renderPassEvent = RenderPassEvent.AfterRenderingOpaques
        };
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        renderer.EnqueuePass(depthRenderPass);
    }
}

