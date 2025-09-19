import torch, torch.nn as nn
from transformers import Dinov2Model

class Model(nn.Module):
	def __init__(self, model_name: str = "facebook/dinov2-small"):
		super().__init__()
		self.dinov2 = Dinov2Model.from_pretrained(model_name)
		self.hidden_dim = self.dinov2.config.hidden_size
		decoder_layer = nn.TransformerDecoderLayer(d_model=self.hidden_dim, nhead=8, dim_feedforward=512, dropout=0.1, batch_first=True)
		self.dinov2_decoder = nn.TransformerDecoder(decoder_layer, num_layers=2)
		self.classifier = nn.Linear(self.hidden_dim, 1)
		self.org_xy_projection = nn.Linear(1024, self.hidden_dim)

	def _compute_dinov2_embeddings(self, rgb):
		xy_embed = self.dinov2(rgb).last_hidden_state[:, 1:, :]
		return xy_embed

	def _to_one_hot_xy(self, indices, batch_size, device):
		one_hot = torch.zeros(batch_size, 1024, device=device)
		one_hot.scatter_(1, indices.unsqueeze(-1), 1.0)
		return one_hot

	def _process_embeddings(self, xy_embed, org_xy_one_hot=None):
		org_memory = None
		if org_xy_one_hot is not None:
			org_memory = self.org_xy_projection(org_xy_one_hot.float()).unsqueeze(1)
		decoder_memory = org_memory if org_memory is not None else xy_embed
		decoded = self.dinov2_decoder(xy_embed, decoder_memory)
		return self.classifier(decoded).squeeze(2)

	def forward(self, rgb, org_xy):
		bs, device = rgb.shape[0], rgb.device
		org_xy_one_hot = self._to_one_hot_xy(org_xy, bs, device)
		xy_embed = self._compute_dinov2_embeddings(rgb)
		org_xy_pred = self._process_embeddings(xy_embed, None)
		action_xy_pred = self._process_embeddings(xy_embed, org_xy_one_hot)
		return {'origin_predictions': org_xy_pred, 'action_predictions': action_xy_pred}
	
	@torch.no_grad()
	def forward_test(self, rgb: torch.Tensor):
		bs, device = rgb.size(0), rgb.device
		xy_embed = self._compute_dinov2_embeddings(rgb)        # [B, 1024, H]
		origin_logits = self._process_embeddings(xy_embed, None)  
		predicted_origins = origin_logits.argmax(dim=1)          # [B]
		origin_one_hot = self._to_one_hot_xy(predicted_origins, bs, device)  # [B, 1024]
		action_logits = self._process_embeddings(xy_embed, origin_one_hot)
		predicted_actions = action_logits.argmax(dim=1)          # [B]

		return {
			'origin_logits':      origin_logits,
			'action_logits':      action_logits,
			'origin_predictions':  predicted_origins,
			'action_predictions':  predicted_actions
		}