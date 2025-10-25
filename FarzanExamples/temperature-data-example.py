# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Example script for for EE 471

import numpy as np
import matplotlib.pyplot as plt

def collect_temperature_data(days):
    """
    Simulates collecting temperature data over a number of days.
    
    Parameters:
    days (int): The number of days to collect data for.
    
    Returns:
    tuple: A tuple containing the day numbers and temperature readings as lists.
    """
    day_numbers = []
    temperatures = []
    
    for day in range(1, days + 1):
        day_numbers.append(day)
        # Simulate temperature reading (random value between 15 and 30)
        temperature = np.random.uniform(15, 30)
        temperatures.append(temperature)
    
    return day_numbers, temperatures

def analyze_and_plot_data(days, temperatures):
    """
    Analyzes the temperature data and creates plots.
    
    Parameters:
    days (list): List of day numbers.
    temperatures (list): List of temperature readings.
    """
    # Convert lists to numpy arrays for easier manipulation
    days_array = np.array(days)
    temp_array = np.array(temperatures)
    
    # Calculate some basic statistics
    avg_temp = np.mean(temp_array)
    max_temp = np.max(temp_array)
    min_temp = np.min(temp_array)
    
    print(f"Average temperature: {avg_temp:.2f}°C")
    print(f"Maximum temperature: {max_temp:.2f}°C")
    print(f"Minimum temperature: {min_temp:.2f}°C")
    
    # Create a line plot of temperature over time
    plt.figure(figsize=(10, 6))
    plt.plot(days_array, temp_array, marker='o')
    plt.title("Daily Temperature Readings")
    plt.xlabel("Day")
    plt.ylabel("Temperature (°C)")
    plt.grid(True)
    plt.show()
    
    # Create a histogram of temperature distribution
    plt.figure(figsize=(8, 5))
    plt.hist(temp_array, bins=10, edgecolor='black')
    plt.title("Distribution of Temperatures")
    plt.xlabel("Temperature (°C)")
    plt.ylabel("Frequency")
    plt.show()

# Main execution
if __name__ == "__main__":
    num_days = 30
    days, temperatures = collect_temperature_data(num_days)
    analyze_and_plot_data(days, temperatures)
