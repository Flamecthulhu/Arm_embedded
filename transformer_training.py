import torch
import torch.nn as nn

class TinyTransformer(nn.Module):
    def __init__(self, vocab_size=100, embed_dim=64, nhead=4, num_layers=2, num_classes=3):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embed_dim)
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim,
            nhead=nhead,
            dim_feedforward=128,
            batch_first=True,
            dropout=0.1
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.classifier = nn.Linear(embed_dim, num_classes)

    def forward(self, x):
        x = self.embedding(x)  # [B, L] -> [B, L, D]
        x = self.encoder(x)    # 自注意力处理
        x = x.mean(dim=1)      # 全局平均池化
        return self.classifier(x)
    
model.eval()
dummy_input = torch.randint(0, 100, (1, 64))  # 批次=1, 序列=64
torch.onnx.export(
    model, dummy_input,
    "tiny_transformer.onnx",
    input_names=["input"],
    output_names=["output"],
    opset_version=11,
    dynamic_axes={"input": {0: "batch"}}  # 可选
)