import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.ensemble import RandomForestRegressor
from sklearn.multioutput import MultiOutputRegressor
import joblib
from scipy.stats import zscore

# Step 1: Load the data
data = pd.read_csv('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/yolo_bags_features.csv')

# Step 2: Handle NaN values
data.fillna(data.median(), inplace=True)

# Calculate correlation matrix for features only
features = ['Relative Obstacle Distance', 'Relative Obstacle Orientation', 'Goal Error X', 'Goal Error Y']
correlation_matrix = data[features].corr()

# Find highly correlated features
threshold = 0.5
features_to_remove = set()

for i in range(len(correlation_matrix.columns)):
    for j in range(i+1, len(correlation_matrix.columns)):
        if abs(correlation_matrix.iloc[i,j]) > threshold:
            # Remove the second feature of the pair
            features_to_remove.add(correlation_matrix.columns[j])

# Remove highly correlated features
selected_features = [f for f in features if f not in features_to_remove]

print("Original features:", features)
print("Features removed due to high correlation:", list(features_to_remove))
print("Selected features:", selected_features)

# Step 3: Handle outliers using Z-score
columns_for_outliers = selected_features + ['Linear Velocity', 'Angular Velocity']
z_scores = np.abs(zscore(data[columns_for_outliers]))
threshold = 3
outliers = (z_scores > threshold).any(axis=1)

# Remove outliers
data_clean = data[~outliers]

# Step 4: Split the data into features and targets
X = data_clean[selected_features]
y = data_clean[['Linear Velocity', 'Angular Velocity']]

# Step 5: Normalize the features
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X.values)

# Step 6: Split data
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# Step 7: Initialize and wrap the model
base_model = RandomForestRegressor(
    n_estimators=100,
    max_depth=15,
    random_state=42
)

model = MultiOutputRegressor(base_model)

# Step 8: Train the model
model.fit(X_train, y_train)

# Step 9: Save the model and scaler
joblib.dump(model, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/rf_model_yolo.pkl')
joblib.dump(scaler, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/rf_scaler_yolo.pkl')

print("Model and Scaler saved successfully.")

# Step 10: Evaluate the model
y_pred = model.predict(X_test)

# Calculate metrics
mse = mean_squared_error(y_test, y_pred)
r2_linear = r2_score(y_test['Linear Velocity'], y_pred[:, 0])
r2_angular = r2_score(y_test['Angular Velocity'], y_pred[:, 1])
mse_linear = mean_squared_error(y_test['Linear Velocity'], y_pred[:, 0])
mse_angular = mean_squared_error(y_test['Angular Velocity'], y_pred[:, 1])

print(f'\nMean Squared Error: {mse}')
print(f'R-squared (Linear Velocity): {r2_linear:.4f}')
print(f'R-squared (Angular Velocity): {r2_angular:.4f}')

# Plot predictions
plt.figure(figsize=(12, 6))

# Linear Velocity
plt.subplot(1, 2, 1)
plt.scatter(y_test['Linear Velocity'], y_pred[:, 0], color='blue', alpha=0.6)
plt.plot([y_test['Linear Velocity'].min(), y_test['Linear Velocity'].max()],
         [y_test['Linear Velocity'].min(), y_test['Linear Velocity'].max()],
         color='red', linestyle='--', linewidth=2)
plt.title(f"Linear Velocity: Predicted vs Actual\nR² = {r2_linear:.4f}, MSE = {mse_linear:.4f}")
plt.xlabel("Actual Linear Velocity")
plt.ylabel("Predicted Linear Velocity")

# Angular Velocity
plt.subplot(1, 2, 2)
plt.scatter(y_test['Angular Velocity'], y_pred[:, 1], color='green', alpha=0.6)
plt.plot([y_test['Angular Velocity'].min(), y_test['Angular Velocity'].max()],
         [y_test['Angular Velocity'].min(), y_test['Angular Velocity'].max()],
         color='red', linestyle='--', linewidth=2)
plt.title(f"Angular Velocity: Predicted vs Actual\nR² = {r2_angular:.4f}, MSE = {mse_angular:.4f}")
plt.xlabel("Actual Angular Velocity")
plt.ylabel("Predicted Angular Velocity")

plt.tight_layout()
plt.show()

# Print model information and feature importances
print("\nRandom Forest Model Information:")
print(f"Number of trees: {base_model.n_estimators}")
print(f"Maximum depth: {base_model.max_depth}")

for i, estimator in enumerate(model.estimators_):
    importances = estimator.feature_importances_
    print(f"\nFeature importances for {'Linear' if i==0 else 'Angular'} Velocity:")
    for feat, imp in zip(selected_features, importances):
        print(f"{feat}: {imp:.4f}")