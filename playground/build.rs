fn main() {
    // Only compile winres on Windows
    #[cfg(windows)]
    {
        let mut res = winres::WindowsResource::new();

        // Set icon if it exists
        let icon_path = "assets/icon.ico";
        if std::path::Path::new(icon_path).exists() {
            res.set_icon(icon_path);
        }

        // Set other metadata
        res.set("ProductName", "Nova Physics Playground");
        res.set("FileDescription", "3D Physics Sandbox with Bevy and Nova");
        res.set("LegalCopyright", "Nova Physics");

        // Compile the resource
        if let Err(e) = res.compile() {
            eprintln!("Warning: Failed to compile Windows resources: {}", e);
        }
    }
}
