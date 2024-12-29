import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
import joblib

# Define the PyTorch MLP model
class MLPModel(nn.Module):
    def __init__(self, input_size, output_size):
        super(MLPModel, self).__init__()
        self.hidden1 = nn.Linear(input_size, 100)
        self.hidden2 = nn.Linear(100, 50)
        self.output = nn.Linear(50, output_size)

    def forward(self, x):
        x = torch.relu(self.hidden1(x))
        x = torch.relu(self.hidden2(x))
        x = self.output(x)
        return x

# Step 1: Load the data
data = pd.read_csv('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/features.csv')

# Step 2: Handle NaN values
data.fillna(data.median(), inplace=True)

# Step 3: Handle outliers using Z-score
from scipy.stats import zscore
z_scores = np.abs(zscore(data[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y', 'Linear Velocity', 'Angular Velocity']]))
threshold = 3
outliers = (z_scores > threshold).all(axis=1)

# Option: Remove outliers
data_clean = data[~outliers]

# Step 4: Split the data into features and targets
X = data_clean[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y']]
y = data_clean[['Linear Velocity', 'Angular Velocity']]

# Step 5: Normalize the features (standardization)
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Step 6: Split data into training and testing sets (80-20 split)
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# Convert data to PyTorch tensors
X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
y_train_tensor = torch.tensor(y_train.values, dtype=torch.float32)

# Step 7: Define the model and optimizer
model = MLPModel(input_size=4, output_size=2)
optimizer = optim.Adam(model.parameters(), lr=0.001)
loss_fn = nn.MSELoss()

# Step 8: Train the model
epochs = 1000
for epoch in range(epochs):
    model.train()
    optimizer.zero_grad()
    y_pred = model(X_train_tensor)
    loss = loss_fn(y_pred, y_train_tensor)
    loss.backward()
    optimizer.step()

    if (epoch + 1) % 100 == 0:
        print(f'Epoch [{epoch + 1}/{epochs}], Loss: {loss.item():.4f}')

# Step 9: Save the model and scaler
torch.save(model.state_dict(), '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/mlp_model.pth')  # Save the PyTorch model
joblib.dump(scaler, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler.pkl')  # Save the scaler using joblib

print("Model and Scaler saved successfully.")

# Step 10: Evaluate the model
# Convert test data to tensors and make predictions
X_test_tensor = torch.tensor(X_test, dtype=torch.float32)
y_test_tensor = torch.tensor(y_test.values, dtype=torch.float32)

model.eval()  # Set the model to evaluation mode
with torch.no_grad():
    y_pred = model(X_test_tensor)

# Calculate MSE
mse = mean_squared_error(y_test_tensor.numpy(), y_pred.numpy())
print(f'Mean Squared Error: {mse}')

# Optionally, print the predictions vs actual values
print("\nPredictions vs Actual Values:")
print(pd.DataFrame({
    "Predicted Linear Velocity": y_pred[:, 0].numpy(),
    "Actual Linear Velocity": y_test['Linear Velocity'].values,
    "Predicted Angular Velocity": y_pred[:, 1].numpy(),
    "Actual Angular Velocity": y_test['Angular Velocity'].values
}))

# Visualize the feature distributions and outliers
for column in ['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y', 'Linear Velocity', 'Angular Velocity']:
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.hist(data[column], bins=30, color='blue', alpha=0.7)
    plt.title(f'{column} Histogram')
    plt.subplot(1, 2, 2)
    plt.boxplot(data[column], vert=False)
    plt.title(f'{column} Boxplot')
    plt.show()
