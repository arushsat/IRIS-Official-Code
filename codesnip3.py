#!/usr/bin/env python3
"""
IRIS Robot - Satellite Image Analysis for Water Source Detection
Uses TensorFlow/Keras for satellite image classification to identify optimal water harvesting zones.
"""

import os
import cv2
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.applications import ResNet50V2
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import matplotlib.pyplot as plt
from PIL import Image
import json
import logging
from datetime import datetime
import argparse

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('iris_satellite_analysis.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class IRISSatelliteAnalyzer:
    """
    Satellite image analyzer for IRIS robot water source detection
    """
    
    def __init__(self, model_path=None, confidence_threshold=0.7):
        self.model = None
        self.confidence_threshold = confidence_threshold
        self.class_names = [
            'Desert', 'Arid_Land', 'Grassland', 'Forest', 
            'Water_Body', 'Wetland', 'Urban', 'Agricultural'
        ]
        self.class_colors = {
            'Desert': [255, 165, 0],      # Orange
            'Arid_Land': [210, 180, 140], # Tan
            'Grassland': [34, 139, 34],   # Forest Green
            'Forest': [0, 100, 0],        # Dark Green
            'Water_Body': [0, 191, 255],  # Deep Sky Blue
            'Wetland': [0, 255, 127],     # Spring Green
            'Urban': [128, 128, 128],     # Gray
            'Agricultural': [255, 215, 0] # Gold
        }
        
        # Load or create model
        if model_path and os.path.exists(model_path):
            self.load_model(model_path)
        else:
            logger.info("No pre-trained model found. Creating new model...")
            self.create_model()
    
    def create_model(self):
        """Create a new CNN model for satellite image classification"""
        try:
            # Use ResNet50V2 as base with transfer learning
            base_model = ResNet50V2(
                weights='imagenet',
                include_top=False,
                input_shape=(224, 224, 3)
            )
            
            # Freeze base model layers
            base_model.trainable = False
            
            # Create new model
            model = keras.Sequential([
                base_model,
                layers.GlobalAveragePooling2D(),
                layers.Dropout(0.5),
                layers.Dense(512, activation='relu'),
                layers.Dropout(0.3),
                layers.Dense(256, activation='relu'),
                layers.Dropout(0.2),
                layers.Dense(len(self.class_names), activation='softmax')
            ])
            
            # Compile model
            model.compile(
                optimizer=keras.optimizers.Adam(learning_rate=0.001),
                loss='categorical_crossentropy',
                metrics=['accuracy']
            )
            
            self.model = model
            logger.info("New model created successfully")
            
        except Exception as e:
            logger.error(f"Error creating model: {e}")
            raise
    
    def load_model(self, model_path):
        """Load a pre-trained model"""
        try:
            self.model = keras.models.load_model(model_path)
            logger.info(f"Model loaded from {model_path}")
        except Exception as e:
            logger.error(f"Error loading model: {e}")
            raise
    
    def save_model(self, model_path):
        """Save the current model"""
        try:
            self.model.save(model_path)
            logger.info(f"Model saved to {model_path}")
        except Exception as e:
            logger.error(f"Error saving model: {e}")
            raise
    
    def preprocess_image(self, image_path):
        """Preprocess satellite image for model input"""
        try:
            # Read image
            if isinstance(image_path, str):
                image = cv2.imread(image_path)
                if image is None:
                    raise ValueError(f"Could not read image from {image_path}")
            else:
                image = image_path
            
            # Convert BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Resize to model input size
            image = cv2.resize(image, (224, 224))
            
            # Normalize pixel values
            image = image.astype(np.float32) / 255.0
            
            # Add batch dimension
            image = np.expand_dims(image, axis=0)
            
            return image
            
        except Exception as e:
            logger.error(f"Error preprocessing image: {e}")
            raise
    
    def analyze_image(self, image_path):
        """Analyze satellite image and return classification results"""
        try:
            # Preprocess image
            processed_image = self.preprocess_image(image_path)
            
            # Make prediction
            predictions = self.model.predict(processed_image, verbose=0)
            
            # Get class probabilities
            class_probs = predictions[0]
            
            # Find best class
            best_class_idx = np.argmax(class_probs)
            best_class = self.class_names[best_class_idx]
            confidence = class_probs[best_class_idx]
            
            # Create results dictionary
            results = {
                'image_path': image_path if isinstance(image_path, str) else 'numpy_array',
                'timestamp': datetime.now().isoformat(),
                'classification': {
                    'primary_class': best_class,
                    'confidence': float(confidence),
                    'all_probabilities': {
                        class_name: float(prob) 
                        for class_name, prob in zip(self.class_names, class_probs)
                    }
                },
                'water_harvesting_potential': self._calculate_water_potential(best_class, confidence),
                'recommendations': self._generate_recommendations(best_class, confidence)
            }
            
            logger.info(f"Image analyzed: {best_class} (confidence: {confidence:.3f})")
            return results
            
        except Exception as e:
            logger.error(f"Error analyzing image: {e}")
            raise
    
    def _calculate_water_potential(self, land_class, confidence):
        """Calculate water harvesting potential based on land classification"""
        # Water harvesting potential scores (0-100)
        potential_scores = {
            'Water_Body': 95,
            'Wetland': 90,
            'Forest': 75,
            'Grassland': 60,
            'Agricultural': 50,
            'Arid_Land': 30,
            'Desert': 20,
            'Urban': 10
        }
        
        base_score = potential_scores.get(land_class, 0)
        
        # Adjust score based on confidence
        adjusted_score = base_score * confidence
        
        # Categorize potential
        if adjusted_score >= 80:
            category = "Excellent"
        elif adjusted_score >= 60:
            category = "Good"
        elif adjusted_score >= 40:
            category = "Moderate"
        elif adjusted_score >= 20:
            category = "Poor"
        else:
            category = "Very Poor"
        
        return {
            'score': round(adjusted_score, 1),
            'category': category,
            'base_score': base_score,
            'confidence_factor': confidence
        }
    
    def _generate_recommendations(self, land_class, confidence):
        """Generate recommendations based on classification results"""
        recommendations = []
        
        if land_class == 'Water_Body':
            recommendations.extend([
                "High water availability detected",
                "Consider deploying water purification systems",
                "Monitor water quality parameters",
                "Ideal location for water collection"
            ])
        elif land_class == 'Wetland':
            recommendations.extend([
                "Good water harvesting potential",
                "Check for seasonal variations",
                "Consider environmental impact",
                "Suitable for atmospheric water generation"
            ])
        elif land_class == 'Forest':
            recommendations.extend([
                "Moderate water potential",
                "Check for natural water sources",
                "Consider tree canopy effects",
                "Good for humidity-based extraction"
            ])
        elif land_class == 'Grassland':
            recommendations.extend([
                "Variable water availability",
                "Check soil moisture levels",
                "Consider seasonal patterns",
                "Moderate potential for water extraction"
            ])
        elif land_class in ['Desert', 'Arid_Land']:
            recommendations.extend([
                "Low water availability expected",
                "Focus on atmospheric water generation",
                "Check for underground water sources",
                "Consider energy-efficient extraction methods"
            ])
        else:
            recommendations.append("Standard water harvesting protocols recommended")
        
        # Add confidence-based recommendations
        if confidence < 0.5:
            recommendations.append("Low confidence - recommend manual verification")
        elif confidence < 0.8:
            recommendations.append("Moderate confidence - consider additional sensors")
        
        return recommendations
    
    def batch_analyze(self, image_directory, output_file=None):
        """Analyze multiple images in a directory"""
        try:
            results = []
            image_extensions = ['.jpg', '.jpeg', '.png', '.tiff', '.bmp']
            
            # Get list of image files
            image_files = [
                f for f in os.listdir(image_directory) 
                if os.path.splitext(f.lower())[1] in image_extensions
            ]
            
            logger.info(f"Found {len(image_files)} images to analyze")
            
            for i, image_file in enumerate(image_files):
                image_path = os.path.join(image_directory, image_file)
                logger.info(f"Analyzing image {i+1}/{len(image_files)}: {image_file}")
                
                try:
                    result = self.analyze_image(image_path)
                    results.append(result)
                except Exception as e:
                    logger.error(f"Error analyzing {image_file}: {e}")
                    continue
            
            # Save results
            if output_file:
                with open(output_file, 'w') as f:
                    json.dump(results, f, indent=2)
                logger.info(f"Results saved to {output_file}")
            
            return results
            
        except Exception as e:
            logger.error(f"Error in batch analysis: {e}")
            raise
    
    def visualize_results(self, image_path, results, output_path=None):
        """Create visualization of analysis results"""
        try:
            # Read original image
            image = cv2.imread(image_path)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Create figure
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
            
            # Original image
            ax1.imshow(image)
            ax1.set_title('Original Satellite Image')
            ax1.axis('off')
            
            # Results visualization
            classes = list(results['classification']['all_probabilities'].keys())
            probabilities = list(results['classification']['all_probabilities'].values())
            
            colors = [self.class_colors.get(cls, [128, 128, 128]) for cls in classes]
            colors = np.array(colors) / 255.0
            
            bars = ax2.barh(classes, probabilities, color=colors)
            ax2.set_xlabel('Probability')
            ax2.set_title('Classification Results')
            ax2.set_xlim(0, 1)
            
            # Add value labels on bars
            for i, (bar, prob) in enumerate(zip(bars, probabilities)):
                ax2.text(prob + 0.01, bar.get_y() + bar.get_height()/2, 
                        f'{prob:.3f}', va='center')
            
            # Highlight primary classification
            primary_idx = classes.index(results['classification']['primary_class'])
            bars[primary_idx].set_edgecolor('red')
            bars[primary_idx].set_linewidth(3)
            
            plt.tight_layout()
            
            if output_path:
                plt.savefig(output_path, dpi=300, bbox_inches='tight')
                logger.info(f"Visualization saved to {output_path}")
            
            plt.show()
            
        except Exception as e:
            logger.error(f"Error creating visualization: {e}")
            raise
    
    def train_model(self, training_data_dir, validation_split=0.2, epochs=10):
        """Train the model with new data"""
        try:
            # Data augmentation for training
            train_datagen = ImageDataGenerator(
                rescale=1./255,
                rotation_range=20,
                width_shift_range=0.2,
                height_shift_range=0.2,
                shear_range=0.2,
                zoom_range=0.2,
                horizontal_flip=True,
                validation_split=validation_split
            )
            
            # Load training data
            train_generator = train_datagen.flow_from_directory(
                training_data_dir,
                target_size=(224, 224),
                batch_size=32,
                class_mode='categorical',
                subset='training'
            )
            
            validation_generator = train_datagen.flow_from_directory(
                training_data_dir,
                target_size=(224, 224),
                batch_size=32,
                class_mode='categorical',
                subset='validation'
            )
            
            # Train model
            history = self.model.fit(
                train_generator,
                validation_data=validation_generator,
                epochs=epochs,
                callbacks=[
                    keras.callbacks.EarlyStopping(patience=3, restore_best_weights=True),
                    keras.callbacks.ReduceLROnPlateau(factor=0.5, patience=2)
                ]
            )
            
            logger.info("Model training completed successfully")
            return history
            
        except Exception as e:
            logger.error(f"Error training model: {e}")
            raise

def main():
    """Main function for command-line usage"""
    parser = argparse.ArgumentParser(description='IRIS Satellite Image Analyzer')
    parser.add_argument('--image', '-i', help='Path to single image for analysis')
    parser.add_argument('--directory', '-d', help='Directory containing images for batch analysis')
    parser.add_argument('--model', '-m', help='Path to pre-trained model')
    parser.add_argument('--output', '-o', help='Output file for results')
    parser.add_argument('--visualize', '-v', action='store_true', help='Create visualization')
    parser.add_argument('--train', '-t', help='Directory for training data')
    parser.add_argument('--epochs', type=int, default=10, help='Number of training epochs')
    
    args = parser.parse_args()
    
    try:
        # Initialize analyzer
        analyzer = IRISSatelliteAnalyzer(model_path=args.model)
        
        if args.train:
            # Training mode
            logger.info("Starting model training...")
            history = analyzer.train_model(args.train, epochs=args.epochs)
            analyzer.save_model('iris_satellite_model.h5')
            
        elif args.directory:
            # Batch analysis mode
            logger.info("Starting batch analysis...")
            results = analyzer.batch_analyze(args.directory, args.output)
            logger.info(f"Batch analysis completed. Processed {len(results)} images.")
            
        elif args.image:
            # Single image analysis
            logger.info("Analyzing single image...")
            results = analyzer.analyze_image(args.image)
            
            # Print results
            print(f"\nAnalysis Results for {args.image}:")
            print(f"Primary Classification: {results['classification']['primary_class']}")
            print(f"Confidence: {results['classification']['confidence']:.3f}")
            print(f"Water Harvesting Potential: {results['water_harvesting_potential']['score']}/100")
            print(f"Category: {results['water_harvesting_potential']['category']}")
            print("\nRecommendations:")
            for rec in results['recommendations']:
                print(f"- {rec}")
            
            # Save results
            if args.output:
                with open(args.output, 'w') as f:
                    json.dump(results, f, indent=2)
                logger.info(f"Results saved to {args.output}")
            
            # Create visualization
            if args.visualize:
                viz_path = args.output.replace('.json', '_visualization.png') if args.output else 'visualization.png'
                analyzer.visualize_results(args.image, results, viz_path)
                
        else:
            parser.print_help()
            
    except Exception as e:
        logger.error(f"Error in main execution: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
