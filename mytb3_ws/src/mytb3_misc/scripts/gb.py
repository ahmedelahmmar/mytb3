import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.multioutput import MultiOutputRegressor
import joblib
from scipy.stats import zscore

# Step 1: Load the data
data = pd.read_csv('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/features.csv')

# Step 2: Handle NaN values
data.fillna(data.median(), inplace=True)

# Step 3: Handle outliers using Z-score
z_scores = np.abs(zscore(data[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y', 'Linear Velocity', 'Angular Velocity']]))
threshold = 3
outliers = (z_scores > threshold).all(axis=1)

# Option: Remove outliers
data_clean = data[~outliers]

# Step 4: Split the data into features and targets
X = data_clean[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y']]
y = data_clean[['Linear Velocity', 'Angular Velocity']]  # Target for both Linear and Angular Velocities

# Step 5: Normalize the features (standardization)
scaler_X = StandardScaler()
X_scaled = scaler_X.fit_transform(X)

# Step 6: Normalize the targets (optional - for inverse transform later)
scaler_y = StandardScaler()
y_scaled = scaler_y.fit_transform(y)

# Step 7: Split data into training and testing sets (80-20 split)
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_scaled, test_size=0.2, random_state=42)

# Step 8: Initialize the GradientBoostingRegressor model and wrap it with MultiOutputRegressor
base_model = GradientBoostingRegressor(
    n_estimators=100,           # Fewer trees for faster training
    learning_rate=0.05,         # Slightly higher learning rate
    max_depth=3,               # Shallower trees
    random_state=42
)


model = MultiOutputRegressor(base_model)

# Step 9: Train the model
model.fit(X_train, y_train)

# Step 10: Save the model and scaler
joblib.dump(model, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/gb_model.pkl')  # Save the model
joblib.dump(scaler_X, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/gb_scaler_X.pkl')  # Save the feature scaler
joblib.dump(scaler_y, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/gb_scaler_y.pkl')  # Save the target scaler

print("Model and Scalers saved successfully.")

# Step 11: Evaluate the model
y_pred_scaled = model.predict(X_test)

# Inverse transform the predictions and the actual targets
y_pred = scaler_y.inverse_transform(y_pred_scaled)
y_test_original = scaler_y.inverse_transform(y_test)

# Calculate MSE
mse = mean_squared_error(y_test_original, y_pred)
print(f'Mean Squared Error: {mse}')

# Step 12: Plot the actual vs predicted data for both linear and angular velocities
plt.figure(figsize=(12, 6))

# Plot for Linear Velocity
plt.subplot(1, 2, 1)
plt.scatter(y_test_original[:, 0], y_pred[:, 0], alpha=0.7, color='blue')
plt.plot([min(y_test_original[:, 0]), max(y_test_original[:, 0])],
         [min(y_test_original[:, 0]), max(y_test_original[:, 0])],
         color='red', linestyle='--', linewidth=2)  # Reference line
plt.title("Actual vs Predicted: Linear Velocity")
plt.xlabel("Actual Linear Velocity")
plt.ylabel("Predicted Linear Velocity")

# Plot for Angular Velocity
plt.subplot(1, 2, 2)
plt.scatter(y_test_original[:, 1], y_pred[:, 1], alpha=0.7, color='green')
plt.plot([min(y_test_original[:, 1]), max(y_test_original[:, 1])],
         [min(y_test_original[:, 1]), max(y_test_original[:, 1])],
         color='red', linestyle='--', linewidth=2)  # Reference line
plt.title("Actual vs Predicted: Angular Velocity")
plt.xlabel("Actual Angular Velocity")
plt.ylabel("Predicted Angular Velocity")

plt.tight_layout()
plt.show()
