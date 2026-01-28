from transformers import AutoTokenizer, AutoModelForSequenceClassification
from torch.utils.data import Dataset, DataLoader
from torch.optim import AdamW
import csv
import torch

class IntentDataset(Dataset):
    def __init__(self, texts, labels, tokenizer, max_len=64):
        self.texts = texts
        self.labels = labels
        self.tokenizer = tokenizer
        self.max_len = max_len

    def __len__(self):
        return len(self.labels)

    def __getitem__(self, idx):
        text = self.texts[idx]
        label = self.labels[idx]
        encoding = self.tokenizer(
            text,
            padding="max_length",
            truncation=True,
            max_length=self.max_len,
            return_tensors="pt"
        )
        item = {key: val.squeeze(0) for key, val in encoding.items()}  # remove batch dim
        item["labels"] = torch.tensor(label)
        return item

intents = []
labels = []

with open("intent.csv", "r", encoding="utf-8") as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        text = row[0]
        label = row[1]
        intents.append(text)
        labels.append(label)

label2id = {l:i for i,l in enumerate(sorted(set(labels)))}
id2label = {i:l for l,i in label2id.items()}
labels_ids = [label2id[l] for l in labels]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model_name = "bert-base-chinese"
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForSequenceClassification.from_pretrained(model_name, num_labels=len(id2label), label2id=label2id).to(device)
dataset = IntentDataset(intents, labels_ids, tokenizer)
loader = DataLoader(dataset, batch_size=16, shuffle=True)
optimizer = AdamW(model.parameters(), lr=5e-5)

model.train()

for epoch in range(10):
    for batch in loader:
        batch = {k:v.to(device) for k,v in batch.items()}
        outputs = model(**batch)
        loss = outputs.loss
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
    print(f"Epoch {epoch} loss: {loss.item():.4f}")

model.eval()

while True:
    text = input("Enter your intent: ")
    if text.lower() == "exit":
        break
    inputs = tokenizer(text, return_tensors="pt").to(device)
    with torch.no_grad():
        logits = model(**inputs).logits
    pred_id = logits.argmax().item()
    pred_label = id2label[pred_id]
    print(pred_label)