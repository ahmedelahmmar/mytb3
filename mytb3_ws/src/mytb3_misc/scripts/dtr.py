#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, r2_score
import joblib
from scipy.stats import zscore

def load_and_preprocess_data(file_path):
    """Load and preprocess the data"""
    # Load the data
    data = pd.read_csv(file_path)
    
    # Handle NaN values
    data.fillna(data.median(), inplace=True)
    return data

def remove_correlated_features(data, features, threshold=0.5):
    """Remove highly correlated features"""
    # Calculate correlation matrix
    correlation_matrix = data[features].corr()
    
    # Find highly correlated features
    features_to_remove = set()
    for i in range(len(correlation_matrix.columns)):
        for j in range(i+1, len(correlation_matrix.columns)):
            if abs(correlation_matrix.iloc[i,j]) > threshold:
                features_to_remove.add(correlation_matrix.columns[j])
    
    # Get selected features
    selected_features = [f for f in features if f not in features_to_remove]
    
    print("Original features:", features)
    print("Features removed due to high correlation:", list(features_to_remove))
    print("Selected features:", selected_features)
    
    return selected_features

def remove_outliers(data, columns, threshold=3):
    """Remove outliers using Z-score"""
    z_scores = np.abs(zscore(data[columns]))
    outliers = (z_scores > threshold).any(axis=1)
    return data[~outliers]

def train_and_evaluate_model(X_train, X_test, y_train, y_test, scaler_y):
    """Train the model and evaluate its performance"""
    # Initialize and train model
    model = DecisionTreeRegressor(random_state=42)
    model.fit(X_train, y_train)
    
    # Make predictions
    y_pred_scaled = model.predict(X_test)
    y_pred = scaler_y.inverse_transform(y_pred_scaled)
    y_test_original = scaler_y.inverse_transform(y_test)
    
    # Calculate metrics
    mse = mean_squared_error(y_test_original, y_pred)
    r2_linear = r2_score(y_test_original[:, 0], y_pred[:, 0])
    r2_angular = r2_score(y_test_original[:, 1], y_pred[:, 1])
    mse_linear = mean_squared_error(y_test_original[:, 0], y_pred[:, 0])
    mse_angular = mean_squared_error(y_test_original[:, 1], y_pred[:, 1])
    
    return model, y_pred, y_test_original, mse, r2_linear, r2_angular, mse_linear, mse_angular

def plot_results(y_test_original, y_pred, r2_linear, r2_angular, mse_linear, mse_angular):
    """Plot the prediction results"""
    plt.figure(figsize=(12, 6))

    # Linear Velocity
    plt.subplot(1, 2, 1)
    plt.scatter(y_test_original[:, 0], y_pred[:, 0], color='blue', alpha=0.6)
    plt.plot([y_test_original[:, 0].min(), y_test_original[:, 0].max()],
             [y_test_original[:, 0].min(), y_test_original[:, 0].max()],
             color='red', linestyle='--', linewidth=2)
    plt.title(f"Linear Velocity: Predicted vs Actual\nR² = {r2_linear:.4f}, MSE = {mse_linear:.4f}")
    plt.xlabel("Actual Linear Velocity")
    plt.ylabel("Predicted Linear Velocity")

    # Angular Velocity
    plt.subplot(1, 2, 2)
    plt.scatter(y_test_original[:, 1], y_pred[:, 1], color='green', alpha=0.6)
    plt.plot([y_test_original[:, 1].min(), y_test_original[:, 1].max()],
             [y_test_original[:, 1].min(), y_test_original[:, 1].max()],
             color='red', linestyle='--', linewidth=2)
    plt.title(f"Angular Velocity: Predicted vs Actual\nR² = {r2_angular:.4f}, MSE = {mse_angular:.4f}")
    plt.xlabel("Actual Angular Velocity")
    plt.ylabel("Predicted Angular Velocity")

    plt.tight_layout()
    plt.show()

def main():
    # Define constants
    DATA_PATH = '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/yolo_bags_features.csv'
    FEATURES = ['Relative Obstacle Distance', 'Relative Obstacle Orientation', 
                'Goal Error X', 'Goal Error Y']
    
    # Load and preprocess data
    data = load_and_preprocess_data(DATA_PATH)
    
    # Remove correlated features
    selected_features = remove_correlated_features(data, FEATURES)
    
    # Remove outliers
    columns_for_outliers = selected_features + ['Linear Velocity', 'Angular Velocity']
    data_clean = remove_outliers(data, columns_for_outliers)
    
    # Prepare features and targets
    X = data_clean[selected_features]
    y = data_clean[['Linear Velocity', 'Angular Velocity']]
    
    # Scale the data
    scaler_X = StandardScaler()
    scaler_y = StandardScaler()
    X_scaled = scaler_X.fit_transform(X.values)
    y_scaled = scaler_y.fit_transform(y.values)
    
    # Split the data
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_scaled, 
                                                        test_size=0.2, random_state=42)
    
    # Train and evaluate model
    model, y_pred, y_test_original, mse, r2_linear, r2_angular, mse_linear, mse_angular = \
        train_and_evaluate_model(X_train, X_test, y_train, y_test, scaler_y)
    
    # Save model and scalers
    joblib.dump(model, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/decision_tree_model_yolo.pkl')
    joblib.dump(scaler_X, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler_X_dtr_yolo.pkl')
    joblib.dump(scaler_y, '/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler_y_dtr_yolo.pkl')
    print("Model and Scalers saved successfully.")
    
    # Print metrics
    print(f'\nMean Squared Error: {mse}')
    print(f'R-squared (Linear Velocity): {r2_linear:.4f}')
    print(f'R-squared (Angular Velocity): {r2_angular:.4f}')
    
    # Plot results
    plot_results(y_test_original, y_pred, r2_linear, r2_angular, mse_linear, mse_angular)
    
    # Print feature importances
    print("\nFeature Importances:")
    for feat, imp in zip(selected_features, model.feature_importances_):
        print(f"{feat}: {imp:.4f}")

if __name__ == '__main__':
    main()