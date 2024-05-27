#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

def preprocess_image(image, color_mask=False,grayscale_colors=0,threshold=150):

    # Apply Gaussian Blur
    blurred_image = cv2.GaussianBlur(image, (15, 15), 100)

    # Convert the image to grayscale
    gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)

    if color_mask:
        # Create masks for red, blue, and white colors
        red_mask = cv2.inRange(image, (0, 0, 200), (100, 100, 255))
        blue_mask = cv2.inRange(image, (200, 0, 0), (255, 100, 100))
        white_mask = cv2.inRange(image, (200, 200, 200), (255, 255, 255))

        # Combine masks
        color_mask = cv2.bitwise_or(cv2.bitwise_or(red_mask, blue_mask), white_mask)

        # Invert the binary image
        color_mask_inv = cv2.bitwise_not(color_mask)

        # Apply the mask to the original image
        masked_image = cv2.bitwise_and(image, image, mask=color_mask_inv)
        return masked_image
    elif grayscale_colors <= 256 and grayscale_colors > 0:
        _, binary_image = cv2.threshold(gray_image, threshold, grayscale_colors-1, cv2.THRESH_BINARY)
        binary_image = cv2.merge((binary_image, binary_image, binary_image))
        return binary_image
    else:
        _, binary_image = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY)
        binary_image = cv2.merge((binary_image, binary_image, binary_image))
        return binary_image
    


def signal_detected_circle(photo):
    # Load the image
    img = cv2.imread(photo)
    img=preprocess_image(img) #################################Mirar si quitar o no

    # Threshold the image
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # Find contours in the image
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the contour of the circle
    circle_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        if 0.7 < circularity < 1.2:
            circle_contour = contour
            break

    # Find the children contours of the circle contour
    children_contours = []
    if circle_contour is not None:
        for contour in contours:
            if cv2.pointPolygonTest(circle_contour, tuple(contour[0][0]), False) > 0:
                children_contours.append(contour)

    print("Children Contours: ", children_contours)
    child_contour=children_contours[0]
    # Calculate the centroid coordinates
    if moments['m00'] != 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Draw the original contour in Blue
        img_child=cv2.drawContours(img, [child_contour], -1, (255, 0, 0), 1)
        # Draw the centroid as a blue dot
        cv2.circle(img_child, (cx, cy), 2, (255, 0, 0), -1)
        #print(f"Child Contour: ({cx} , {cy})")
        # Print the area of the first child contour
        area = cv2.contourArea(child_contour)
        #print(f"Area of child Contour {i+1}: {area}")
        cv2.imshow('Child', img_child)
        #cv2.waitKey(0)
    
    parent_contour = circle_contour
    # Calculate the moments for the Parent contour
    moments = cv2.moments(parent_contour)

    # Calculate the centroid coordinates
    if moments['m00'] != 0:
        px = int(moments['m10'] / moments['m00'])
        py = int(moments['m01'] / moments['m00'])

        # Draw the first parent contour in red
        img_parent=cv2.drawContours(img, [parent_contour], -1, (0, 0, 255), 1)
        # Draw the centroid as a red dot
        cv2.circle(img_parent, (px, py), 2, (0, 0, 255), -1)
        #print(f"Parent Contour: ({px} , {py})")
        cv2.imshow('Parent', img_parent)
        #cv2.waitKey(0)


    # Print the detection
    dif=cx-px
           
    # Display the image with contours and centroids
    cv2.imshow('Contours with Centroids', cv2.resize(img, (800, 600)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    rospy.loginfo("Difference " + str(dif))
    
    if dif > 0:
        signal = "right"
    else:
        signal = "left"
    rospy.loginfo("Centroids " + str(dif))
    return signal





def signal_detectedPre(photo):
    img = cv2.imread(photo)
    img=preprocess_image(img)
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help with contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours and hierarchy
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #print("Contours:\n"+str(contours))

    # Initialize variables to keep track of the contour with the highest hierarchy_info[3] value
    max_parent_index = -1
    max_hierarchy_value = -1


    # Iterate through contours and hierarchy
    for i in range(len(contours)):
        #print (f"Contour: {i}")
        hierarchy_info = hierarchy[0][i]
        cv2.drawContours(img, contours[i], -1, (0,255,0), 1)
        cv2.imshow('imagen',img)
        #cv2.waitKey(0)
        
        # Check if the contour has a parent
        if hierarchy_info[3] != -1:
            # Update the variables if the current contour has a higher hierarchy_info[3] value
            if hierarchy_info[3] > max_hierarchy_value:
                max_hierarchy_value = hierarchy_info[3]
                max_parent_index = i#index starts with 0

    #print ('hierarchy=\n',hierarchy)
    #print (f"Index: {max_parent_index} Value: {max_hierarchy_value}")
    #cv2.waitKey(0)

    # Access the first child contour using hierarchy_info[max_hierarchy_value]
    child_contour = contours[max_parent_index]
    parent_contour = contours[max_parent_index-2]# there are 2 contours in arrow (inner and outer)
    # Calculate the moments for the child contour
    moments = cv2.moments(child_contour)
    


    # Calculate the centroid coordinates
    if moments['m00'] != 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Draw the original contour in Blue
        img_child=cv2.drawContours(img, [child_contour], -1, (255, 0, 0), 1)
        # Draw the centroid as a blue dot
        cv2.circle(img_child, (cx, cy), 2, (255, 0, 0), -1)
        #print(f"Child Contour: ({cx} , {cy})")
        # Print the area of the first child contour
        area = cv2.contourArea(child_contour)
        #print(f"Area of child Contour {i+1}: {area}")
        cv2.imshow('Child', img_child)
        #cv2.waitKey(0)
        
    # Calculate the moments for the Parent contour
    moments = cv2.moments(parent_contour)

    # Calculate the centroid coordinates
    if moments['m00'] != 0:
        px = int(moments['m10'] / moments['m00'])
        py = int(moments['m01'] / moments['m00'])

        # Draw the first parent contour in red
        img_parent=cv2.drawContours(img, [parent_contour], -1, (0, 0, 255), 1)
        # Draw the centroid as a red dot
        cv2.circle(img_parent, (px, py), 2, (0, 0, 255), -1)
        #print(f"Parent Contour: ({px} , {py})")
        cv2.imshow('Parent', img_parent)
        #cv2.waitKey(0)


    # Print the detection
    dif=cx-px
           
    # Display the image with contours and centroids
    cv2.imshow('Contours with Centroids', cv2.resize(img, (800, 600)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    rospy.loginfo("Difference " + str(dif))
    
    if dif > 0:
        signal = "right"
    else:
        signal = "left"
    rospy.loginfo("Centroids " + str(dif))
    return signal


import cv2
import numpy as np

def signal_detected_fast(photo):
    img = cv2.imread(photo)
    height, width = img.shape[:2]

    # Calculate the ratio to resize the image width to 720 pixels
    resize_ratio = 400 / width

    # Resize the image
    resized_image = cv2.resize(img, (int(width * resize_ratio), int(height * resize_ratio)))



    blurred_image = cv2.GaussianBlur(resized_image, (15, 15), 100)  # Puedes ajustar el tamaño del kernel según sea necesario


    # Convierte la imagen a escala de grises
    gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)

    # Aplica la binarización
    _, binary_image = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY)

    binary_image = cv2.merge((binary_image, binary_image, binary_image))
    # Convert the image to grayscale
    gray = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)
    #gray=img

    # Apply GaussianBlur to reduce noise and help with contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 20)

    # Applying the Canny Edge filter
    edges = cv2.Canny(blurred, 100, 200, apertureSize=5, L2gradient=True)

    # Find contours and hierarchy
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Initialize variables to keep track of the contour with the highest hierarchy_info[3] value
    max_parent_index = -1
    max_hierarchy_value = -1

    # Iterate through contours and hierarchy
    for i, (contour, hierarchy_info) in enumerate(zip(contours, hierarchy[0])):
        cv2.drawContours(img, contour, -1, (0, 255, 0), 1)

        # Check if the contour has a parent
        if hierarchy_info[3] != -1:
            # Update the variables if the current contour has a higher hierarchy_info[3] value
            if hierarchy_info[3] > max_hierarchy_value:
                max_hierarchy_value = hierarchy_info[3]
                max_parent_index = i

    # Access the parent contour and child contour
    parent_contour = contours[max_parent_index - 2] if max_parent_index >= 2 else None
    child_contour = contours[max_parent_index] if max_parent_index != -1 else None

    px=0

    # Calculate the moments for the parent contour
    if parent_contour is not None:
        moments = cv2.moments(parent_contour)
        if moments['m00'] != 0:
            px = int(moments['m10'] / moments['m00'])
            py = int(moments['m01'] / moments['m00'])

            # Draw the parent contour in red
            cv2.drawContours(img, [parent_contour], -1, (0, 0, 255), 1)
            # Draw the centroid as a red dot
            cv2.circle(img, (px, py), 2, (0, 0, 255), -1)

    # Calculate the moments for the child contour
    if child_contour is not None:
        moments = cv2.moments(child_contour)
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

            # Draw the child contour in blue
            cv2.drawContours(img, [child_contour], -1, (255, 0, 0), 1)
            # Draw the centroid as a blue dot
            cv2.circle(img, (cx, cy), 2, (255, 0, 0), -1)

            # Print the detection
            dif = cx - px if parent_contour is not None else 0
            print("Difference: " + str(dif))

            # Display the image with contours and centroids
            cv2_imshow(cv2.resize(img, (800, 600)))
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # Determine the signal direction
            signal = "right" if dif > 0 else "left"
            print("Signal: " + signal)

            return signal

    return None


class KerasImageClassifier:
    def __init__(self, model_path, labels_path):
        self.model = load_model(model_path, compile=False)
        self.class_names = open(labels_path, "r").readlines()

    def predict_image(self, image):
        # Resize the raw image into (224-height,224-width) pixels
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)

        # Make the image a numpy array and reshape it to the models input shape.
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)

        # Normalize the image array
        image = (image / 127.5) - 1

        # Predicts the model
        prediction = self.model.predict(image)
        index = np.argmax(prediction)
        class_name = self.class_names[index]
        confidence_score = prediction[0][index]

        return class_name[2:], confidence_score
    
def combined_detection(photo,model,threshold=0.9):
    img = cv2.imread(photo)
    
    class_pred,confidence = model.predict_image(img)
    
    if confidence > threshold:
        
        if class_pred not in ["Derecha","Izquierda","Flecha"]:
            return class_pred
        else:
            
            height, width = img.shape[:2]

            # Calculate the ratio to resize the image width to 720 pixels
            resize_ratio = 400 / width

            # Resize the image
            resized_image = cv2.resize(img, (int(width * resize_ratio), int(height * resize_ratio)))



            blurred_image = cv2.GaussianBlur(resized_image, (15, 15), 100)  # Puedes ajustar el tamaño del kernel según sea necesario


            # Convierte la imagen a escala de grises
            gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)

            # Aplica la binarización
            _, binary_image = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY)

            binary_image = cv2.merge((binary_image, binary_image, binary_image))
            # Convert the image to grayscale
            gray = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)
            #gray=img

            # Apply GaussianBlur to reduce noise and help with contour detection
            blurred = cv2.GaussianBlur(gray, (5, 5), 20)

            # Applying the Canny Edge filter
            edges = cv2.Canny(blurred, 100, 200, apertureSize=5, L2gradient=True)

            # Find contours and hierarchy
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            # Initialize variables to keep track of the contour with the highest hierarchy_info[3] value
            max_parent_index = -1
            max_hierarchy_value = -1

            # Iterate through contours and hierarchy
            for i, (contour, hierarchy_info) in enumerate(zip(contours, hierarchy[0])):
                cv2.drawContours(img, contour, -1, (0, 255, 0), 1)

                # Check if the contour has a parent
                if hierarchy_info[3] != -1:
                    # Update the variables if the current contour has a higher hierarchy_info[3] value
                    if hierarchy_info[3] > max_hierarchy_value:
                        max_hierarchy_value = hierarchy_info[3]
                        max_parent_index = i

            # Access the parent contour and child contour
            parent_contour = contours[max_parent_index - 2] if max_parent_index >= 2 else None
            child_contour = contours[max_parent_index] if max_parent_index != -1 else None

            px=0

            # Calculate the moments for the parent contour
            if parent_contour is not None:
                moments = cv2.moments(parent_contour)
                if moments['m00'] != 0:
                    px = int(moments['m10'] / moments['m00'])
                    py = int(moments['m01'] / moments['m00'])

                    # Draw the parent contour in red
                    cv2.drawContours(img, [parent_contour], -1, (0, 0, 255), 1)
                    # Draw the centroid as a red dot
                    cv2.circle(img, (px, py), 2, (0, 0, 255), -1)

            # Calculate the moments for the child contour
            if child_contour is not None:
                moments = cv2.moments(child_contour)
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])

                    # Draw the child contour in blue
                    cv2.drawContours(img, [child_contour], -1, (255, 0, 0), 1)
                    # Draw the centroid as a blue dot
                    cv2.circle(img, (cx, cy), 2, (255, 0, 0), -1)

                    # Print the detection
                    dif = cx - px if parent_contour is not None else 0
                    print("Difference: " + str(dif))

                    # Display the image with contours and centroids
                    cv2.imshow(cv2.resize(img, (800, 600)))
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    # Determine the signal direction
                    signal = "right" if dif > 0 else "left"
                    print("Signal: " + signal)

                    return signal

            return None

if __name__ == '__main__':
    # Read signal
    #image = cv2.imread('left.png')
    
    #classifier = KerasImageClassifier("/content/keras_model.h5", "/content/labels.txt") //Poner el path absoluto
    
    # class_name, confidence_score = classifier.predict_image(image)
    #     print("Class:", class_name, end="")
    #     print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")

    photo = "left.png"
    signal = signal_detected_fast(photo)
    print("Signal detected: ", signal)
    #cv2.imshow('Signal',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
