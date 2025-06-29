from sympy import symbols, Matrix, sin, cos, sqrt, simplify, S

# Define symbolic variables
omega_x, omega_y, omega_z, dt = symbols('omega_x omega_y omega_z dt')

# Define the sigma vector components (omega * dt)
sigma_x = omega_x * dt
sigma_y = omega_y * dt
sigma_z = omega_z * dt

# Construct the Omega(sigma) matrix
Omega_sigma = Matrix([
    [0, -sigma_x, -sigma_y, -sigma_z],
    [sigma_x, 0, sigma_z, -sigma_y],
    [sigma_y, -sigma_z, 0, sigma_x],
    [sigma_z, sigma_y, -sigma_x, 0]
])

# Compute the matrix exponential
# M_exp = (S(1)/2 * Omega_sigma).exp() # This is the more direct way, but often results in complex output
                                     # The structure of Omega(sigma) is known, so often better to simplify first

# For skew-symmetric matrices, the exponential has a known form.
# It's related to Rodrigues' formula.
# Let's try to derive it using properties if SymPy's direct exp() is too complex.
# The magnitude of sigma vector
norm_sigma_sq = sigma_x**2 + sigma_y**2 + sigma_z**2
norm_sigma = sqrt(norm_sigma_sq)

# Define common terms in the simplified matrix exponential form
half_norm_sigma = norm_sigma / 2
cos_val = cos(half_norm_sigma)
sin_val = sin(half_norm_sigma)

# Handle the sin(x)/x term carefully for symbolic x=0
# SymPy's sinc function is often sinc(pi*x) = sin(pi*x)/(pi*x)
# So we'll use sin_val / norm_sigma directly, and handle the norm_sigma=0 case conceptually
sinc_val_half = sin_val / norm_sigma

# Construct the 3x3 skew-symmetric matrix for cross product [sigma x]
skew_sigma = Matrix([
    [0, -sigma_z, sigma_y],
    [sigma_z, 0, -sigma_x],
    [-sigma_y, sigma_x, 0]
])

# Construct the expected matrix based on the formula from your image
# Note: Need to convert 1/2 from float to SymPy rational using S(1)/2
exp_matrix_expected = Matrix([
    [cos_val, -sinc_val_half * sigma_x, -sinc_val_half * sigma_y, -sinc_val_half * sigma_z],
    [sinc_val_half * sigma_x, cos_val - sinc_val_half * skew_sigma[0,0], sinc_val_half * skew_sigma[0,2] + cos_val * S(0), sinc_val_half * skew_sigma[0,1] + cos_val * S(0)], # This part requires careful expansion
    [sinc_val_half * sigma_y, sinc_val_half * skew_sigma[1,2] + cos_val * S(0), cos_val - sinc_val_half * skew_sigma[1,1], sinc_val_half * skew_sigma[1,0] + cos_val * S(0)],
    [sinc_val_half * sigma_z, sinc_val_half * skew_sigma[2,1] + cos_val * S(0), sinc_val_half * skew_sigma[2,0] + cos_val * S(0), cos_val - sinc_val_half * skew_sigma[2,2]]
])

# A more systematic way for the 3x3 block using matrix operations
I3 = Matrix.eye(3)
bottom_right_block = cos_val * I3 - sinc_val_half * skew_sigma

exp_matrix_reconstructed = Matrix.vstack(
    Matrix.hstack(Matrix([[cos_val]]), -sinc_val_half * Matrix([[sigma_x, sigma_y, sigma_z]])),
    Matrix.hstack(sinc_val_half * Matrix([[sigma_x], [sigma_y], [sigma_z]]), bottom_right_block)
)

# Print the resulting matrix (it can be quite large)
print("Derived Matrix Exponential:")
print(exp_matrix_reconstructed)