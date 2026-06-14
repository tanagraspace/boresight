# VHS tape that records the boresight demo GIF used on the project page.
# Render with:  vhs docs/boresight-demo-40x.tape   (run from the repo root)
# Produces:     docs/boresight-demo-40x.gif
Output "/Users/georgeslabreche/Development/other/boresight/docs/boresight-demo-40x.gif"

Set Shell "bash"
Set FontSize 13
Set Width 1320
Set Height 780
Set Padding 12
Set Framerate 24

# Boot the tool hidden so the GIF opens directly on the live TUI rather than on
# the shell prompt and the command being typed.
Hide
Type "cd /Users/georgeslabreche/Development/other/boresight && clear"
Enter
Type "./target/release/boresight --config examples/config/pass.toml --playback 40"
Enter
Sleep 500ms
Show

# Record the pass replaying at 40x; the marker, scope, and chart cursor advance
# together on their own (no key presses needed).
Sleep 9s

# Show the analysis tables, then return to the main view.
Type "t"
Sleep 3s
Type "t"
Sleep 1s

# Quit hidden so the recording ends on the last TUI frame (clean loop).
Hide
Type "q"
Sleep 300ms
