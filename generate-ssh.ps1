param (
    [Parameter(Mandatory=$true)]
    [string]$hostname
)

# Set the directory path
$dir = "ssh_keys/$hostname"

# Create the directory
New-Item -ItemType Directory -Force -Path $dir

# Generate the SSH keys with the hostname as the comment
ssh-keygen -q -t rsa -b 4096 -f "$dir/ssh_host_rsa_key" -N "" -C $hostname
ssh-keygen -q -t dsa -f "$dir/ssh_host_dsa_key" -N "" -C $hostname
ssh-keygen -q -t ecdsa -f "$dir/ssh_host_ecdsa_key" -N "" -C $hostname
ssh-keygen -q -t ed25519 -f "$dir/ssh_host_ed25519_key" -N "" -C $hostname

# Change permissions (not sure if this is necessary)
icacls "$dir/ssh_host_rsa_key" /grant:r Everyone:F
icacls "$dir/ssh_host_dsa_key" /grant:r Everyone:F
icacls "$dir/ssh_host_ecdsa_key" /grant:r Everyone:F
icacls "$dir/ssh_host_ed25519_key" /grant:r Everyone:F

Write-Host "SSH keys generated in $dir with comment $hostname"
