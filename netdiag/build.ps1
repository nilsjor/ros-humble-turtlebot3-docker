# List of images
$images = @(
    "samiemostafavi/sshd-image",
    "samiemostafavi/sdr-tools"
)

# Loop through each image
foreach ($image in $images) {
    # Strip away the author to get the image name
    $imageParts = $image.Split('/')
    $imageName = $imageParts[1]

    # Define the new image name
    $newImageName = "nilsjor/$imageName"

    # Build the new image with the appropriate BASE_IMAGE argument
    docker build --push --build-arg BASE_IMAGE=$image -t $newImageName .
}