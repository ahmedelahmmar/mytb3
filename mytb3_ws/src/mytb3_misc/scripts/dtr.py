#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
import joblib
from scipy.stats import zscore

# Step 1: Load the data
data = pd.read_csv('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/features.csv')

# Step 2: Handle NaN values
data.fillna(data.median(), inplace=True)

# Step 3: Handle outliers using Z-score
z_scores = np.abs(zscore(data[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y', 'Linear Velocity', 'Angular Velocity']]))
threshold = 3
outliers = (z_scores > threshold).any(axis=1)

# Remove outliers
data_clean = data[~outliers]

# Step 4: Split the data into features and targets
X = data_clean[['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y']]
y = data_clean[['Linear Velocity', 'Angular Velocity']]

# Step 5: Normalize the features
scaler_X = StandardScaler()
scaler_y = StandardScaler()
X_scaled = scaler_X.fit_transform(X)
y_scaled = scaler_y.fit_transform(y)

# Step 6: Split data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_scaled, test_size=0.2, random_state=42)

# Step 7: Define and train the Decision Tree Regressor
model = DecisionTreeRegressor(random_state=42)
model.fit(X_train, y_train)

# Step 8: Save the model and scalers
joblib.dump(model, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/decision_tree_model.pkl')
joblib.dump(scaler_X, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler_X_dtr.pkl')
joblib.dump(scaler_y, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler_y_dtr.pkl')

print("Model and Scalers saved successfully.")

# Step 9: Evaluate the model
y_pred_scaled = model.predict(X_test)

# Inverse transform the predictions and actual values
y_pred = scaler_y.inverse_transform(y_pred_scaled)
y_test_original = scaler_y.inverse_transform(y_test)

# Calculate MSE on the original scale
mse = mean_squared_error(y_test_original, y_pred)
print(f'Mean Squared Error (original scale): {mse}')

# Print predictions vs actual values
predictions_df = pd.DataFrame({
    "Predicted Linear Velocity": y_pred[:, 0],
    "Actual Linear Velocity": y_test_original[:, 0],
    "Predicted Angular Velocity": y_pred[:, 1],
    "Actual Angular Velocity": y_test_original[:, 1]
})
print("\nPredictions vs Actual Values:")
print(predictions_df)

# Visualize feature distributions and outliers
for column in ['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y', 'Linear Velocity', 'Angular Velocity']:
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.hist(data[column], bins=30, color='blue', alpha=0.7)
    plt.title(f'{column} Histogram')
    plt.subplot(1, 2, 2)
    plt.boxplot(data[column], vert=False)
    plt.title(f'{column} Boxplot')
    plt.show()