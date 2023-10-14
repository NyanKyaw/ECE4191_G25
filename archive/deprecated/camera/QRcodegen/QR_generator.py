import qrcode

# Define the data
data = "3, 4"

# Create a QR code instance
qr = qrcode.QRCode(
    version = 1,
    error_correction = qrcode.constants.ERROR_CORRECT_H,
    box_size = 10,
    border = 4
)

# Add data to the QR code
qr.add_data(data)
qr.make(fit = True)

# Create an image from the QR code instance
img = qr.make_image(fill = 'black', back_color = 'white')

# Save the image
img.save('QR_code')
