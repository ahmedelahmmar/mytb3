import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
from sklearn.ensemble import RandomForestRegressor
from sklearn.multioutput import MultiOutputRegressor
import joblib
from scipy.stats import zscore

# Step 1: Load the data
data = pd.read_csv('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/features.csv')

# Step 2: Handle NaN values
data.fillna(data.median(), inplace=True)

# Step 3: Handle outliers using Z-score
z_scores = np.abs(zscore(data[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 
                               'Goal Error X', 'Goal Error Y', 'Linear Velocity', 'Angular Velocity']]))
threshold = 3
outliers = (z_scores > threshold).any(axis=1)

# Remove outliers
data_clean = data[~outliers]

# Step 4: Split the data into features and targets
X = data_clean[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y']]
y = data_clean[['Linear Velocity', 'Angular Velocity']]  # Targets for both Linear and Angular Velocities

# Step 5: Normalize the features (standardization)
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Step 6: Split data into training and testing sets (80-20 split)
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# Step 7: Initialize the RandomForestRegressor model and wrap it with MultiOutputRegressor
base_model = RandomForestRegressor(
    n_estimators=100,  # Number of trees in the forest
    max_depth=10,  # Maximum depth of each tree
    random_state=42
)

model = MultiOutputRegressor(base_model)

# Step 8: Train the model
model.fit(X_train, y_train)

# Step 9: Save the model and scaler
joblib.dump(model, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/rf_model.pkl')  # Save the model
joblib.dump(scaler, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/rf_scaler.pkl')  # Save the scaler using joblib

print("Model and Scaler saved successfully.")

# Step 10: Evaluate the model
y_pred = model.predict(X_test)

# Step 11: Inverse scale the predictions and actual target values
y_pred_original = y_pred  # No inverse transform needed as the targets were not scaled
y_test_original = y_test.to_numpy()

# Calculate MSE
mse = mean_squared_error(y_test_original, y_pred_original)
print(f'Mean Squared Error: {mse}')

# Print predictions vs actual values
print("\nPredictions vs Actual Values:")
comparison_df = pd.DataFrame({
    "Predicted Linear Velocity": y_pred_original[:, 0],
    "Actual Linear Velocity": y_test_original[:, 0],
    "Predicted Angular Velocity": y_pred_original[:, 1],
    "Actual Angular Velocity": y_test_original[:, 1]
})
print(comparison_df)

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

# Step 12: Plot predicted vs actual values
plt.figure(figsize=(12, 6))

# Linear Velocity
plt.subplot(1, 2, 1)
plt.scatter(y_test_original[:, 0], y_pred_original[:, 0], color='blue', alpha=0.6, label='Predictions')
plt.plot([min(y_test_original[:, 0]), max(y_test_original[:, 0])], 
         [min(y_test_original[:, 0]), max(y_test_original[:, 0])], 
         color='red', linestyle='--', label='Perfect Fit')
plt.title("Linear Velocity: Predicted vs Actual")
plt.xlabel("Actual Linear Velocity")
plt.ylabel("Predicted Linear Velocity")
plt.legend()

# Angular Velocity
plt.subplot(1, 2, 2)
plt.scatter(y_test_original[:, 1], y_pred_original[:, 1], color='green', alpha=0.6, label='Predictions')
plt.plot([min(y_test_original[:, 1]), max(y_test_original[:, 1])], 
         [min(y_test_original[:, 1]), max(y_test_original[:, 1])], 
         color='red', linestyle='--', label='Perfect Fit')
plt.title("Angular Velocity: Predicted vs Actual")
plt.xlabel("Actual Angular Velocity")
plt.ylabel("Predicted Angular Velocity")
plt.legend()

plt.tight_layout()
plt.show()
