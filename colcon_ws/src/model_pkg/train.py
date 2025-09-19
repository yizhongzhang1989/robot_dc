import os, glob
import torch, torch.nn as nn, torch.optim as optim
from torch.utils.data import DataLoader, random_split
from data_loader import RobotDataset
from model import Model
from tqdm import tqdm

def find_latest_checkpoint(checkpoint_dir="result"):
	if not os.path.exists(checkpoint_dir): return None
	files = glob.glob(os.path.join(checkpoint_dir, "model_epoch_*.pth"))
	if not files: return None
	latest_file = max(files, key=lambda f: int(os.path.basename(f).split('_')[2].split('.')[0]) if len(os.path.basename(f).split('_'))>2 else -1)
	return latest_file

def load_checkpoint(model, optimizer, checkpoint_path, device):
	print(f"Loading checkpoint from {checkpoint_path}")
	checkpoint = torch.load(checkpoint_path, map_location=device)
	model.load_state_dict(checkpoint)
	return 0

def save_model_checkpoint(model, epoch, output_dir="result"):
	os.makedirs(output_dir, exist_ok=True)
	name = f'model_epoch_{epoch+1}.pth' if isinstance(epoch, int) else f'model_epoch_{epoch}.pth'
	path = os.path.join(output_dir, name)
	torch.save(model.state_dict(), path)
	print(f"Model checkpoint saved at {path}")

def calculate_loss(predictions, targets, criterion):
	predictions = predictions.view(-1, predictions.shape[-1])
	targets = targets.view(-1)
	loss = criterion(predictions, targets)
	preds = torch.argmax(predictions, dim=1)
	acc = (preds == targets).float().mean().item()
	return loss, acc

def evaluate_model(epoch, global_step, model, dataloader, criterion, device, is_training=False, optimizer=None, split="train"):
	if is_training: model.train()
	else: model.eval()
	total_loss=total_origin=total_action=0.0; batches=0
	pbar = tqdm(dataloader, desc='Training' if is_training else 'Validation')
	for batch_idx, batch in enumerate(pbar):
		images = batch['img'].to(device)
		origin_grid = batch['origin_grid'].to(device)
		action_grid = batch['action_grid'].to(device)
		outputs = model(images, origin_grid)
		loss_o, acc_o = calculate_loss(outputs['origin_predictions'], origin_grid, criterion)
		loss_a, acc_a = calculate_loss(outputs['action_predictions'], action_grid, criterion)
		batch_loss = loss_o + loss_a
		total_origin += acc_o; total_action += acc_a
		if is_training:
			optimizer.zero_grad(); batch_loss.backward(); optimizer.step()
			if batch_idx % 1000 == 0: save_model_checkpoint(model, epoch)
			global_step += 1
		total_loss += batch_loss.item(); batches += 1
		pbar.set_postfix({'Loss': batch_loss.item(), 'Origin Acc': f'{acc_o:.3f}', 'Action Acc': f'{acc_a:.3f}'})
	return {
		'loss': (total_loss / batches) if batches else 0.0,
		'origin_acc': (total_origin / batches) if batches else 0.0,
		'action_acc': (total_action / batches) if batches else 0.0,
	}

def train():
	batch_size=2; lr=3e-5; num_epochs=100
	device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
	dataset = RobotDataset(data_root="dataset/saved_data", is_training=True)
	indices = [i for i,e
	 in enumerate(dataset.index)]
	dataset = torch.utils.data.Subset(dataset, indices)
	train_size = int(0.9 * len(dataset)); val_size = len(dataset) - train_size
	train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
	train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4)
	val_dataloader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4)
	model = Model().to(device)
	criterion = nn.CrossEntropyLoss()
	optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
	start_epoch = 0; best_val_loss = float('inf')
	latest_checkpoint = find_latest_checkpoint()
	if latest_checkpoint:
		start_epoch = load_checkpoint(model, optimizer, latest_checkpoint, device)
		print(f"Loaded checkpoint. Resuming from epoch {start_epoch + 1}")
	global_step = start_epoch * len(train_dataloader)
	for epoch in range(start_epoch, num_epochs):
		train_metrics = evaluate_model(epoch, global_step, model, train_dataloader, criterion, device, is_training=True, optimizer=optimizer, split="train")
		with torch.no_grad():
			val_metrics = evaluate_model(epoch, global_step, model, val_dataloader, criterion, device, is_training=False, split="val")
		print(f"Epoch {epoch+1}/{num_epochs}")
		print(f"  Train - Loss: {train_metrics['loss']:.4f}, Origin: {train_metrics['origin_acc']:.4f}, Action: {train_metrics['action_acc']:.4f}")
		print(f"  Val   - Loss: {val_metrics['loss']:.4f}, Origin: {val_metrics['origin_acc']:.4f}, Action: {val_metrics['action_acc']:.4f}")
		if val_metrics['loss'] < best_val_loss:
			best_val_loss = val_metrics['loss']
			save_model_checkpoint(model, epoch)
			print(f'  *** New best model! Val loss: {val_metrics["loss"]:.4f} ***')
	save_model_checkpoint(model, 'final')
	print(f"Training completed! Best val loss: {best_val_loss:.4f}")

if __name__ == "__main__":
	train()