import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt

# Step 1: Read the CSV file with the updated absolute path
df = pd.read_csv('/home/rpi5/tflite-custom-object-bookworm-main/detection_data.csv')

# Step 2: Clean and prepare data
cleaned_df = df[['Date', 'Time', 'Width', 'Height']]  # Select relevant columns

# Convert 'Time' column to datetime format
cleaned_df['Time'] = pd.to_datetime(cleaned_df['Time'], errors='coerce')

# Drop rows with NaN values if necessary
cleaned_df = cleaned_df.dropna()

# Convert 'Time' column to Unix timestamp (seconds since epoch)
cleaned_df['Time'] = cleaned_df['Time'].astype(int) // 10**9  # Convert to seconds since epoch

# Step 3: Implement your machine learning model (e.g., Linear Regression)
# Split data into features (X) and target variables (y_width for Width, y_height for Height)
X = cleaned_df[['Time']]  # Time column
y_width = cleaned_df['Width']  # Width column
y_height = cleaned_df['Height']  # Height column

# Split data into training and testing sets
X_train, X_test, y_width_train, y_width_test = train_test_split(X, y_width, test_size=0.2, random_state=42)
X_train, X_test, y_height_train, y_height_test = train_test_split(X, y_height, test_size=0.2, random_state=42)

# Initialize and train linear regression models
model_width = LinearRegression()
model_width.fit(X_train, y_width_train)

model_height = LinearRegression()
model_height.fit(X_train, y_height_train)

# Convert X_test['Time'] to Unix timestamp (seconds since epoch)
X_test['Time'] = X_test['Time'].astype(int) // 10**9  # Convert to seconds since epoch

# Predictions
y_width_pred = model_width.predict(X_test)
y_height_pred = model_height.predict(X_test)

# Evaluation
mse_width = mean_squared_error(y_width_test, y_width_pred)
mse_height = mean_squared_error(y_height_test, y_height_pred)

print(f'Mean Squared Error (Width): {mse_width}')
print(f'Mean Squared Error (Height): {mse_height}')

# Plotting predictions
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.scatter(X_test['Time'], y_width_test, color='blue', label='Actual Width')
plt.plot(X_test['Time'], y_width_pred, color='red', linewidth=2, label='Predicted Width')
plt.xlabel('Time (Unix Timestamp)')
plt.ylabel('Width')
plt.title('Width Prediction')
plt.legend()

plt.subplot(1, 2, 2)
plt.scatter(X_test['Time'], y_height_test, color='blue', label='Actual Height')
plt.plot(X_test['Time'], y_height_pred, color='red', linewidth=2, label='Predicted Height')
plt.xlabel('Time (Unix Timestamp)')
plt.ylabel('Height')
plt.title('Height Prediction')
plt.legend()

plt.tight_layout()
plt.show()
