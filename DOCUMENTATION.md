# Documentation Setup Guide

This project now includes comprehensive documentation available in two formats:

## Documentation Locations

### GitHub Wiki
Located in: `/wiki/` folder

Access the wiki documentation directly from GitHub:
1. Go to your GitHub repository
2. Click the "Wiki" tab at the top
3. Use the sidebar for navigation

**Wiki Files:**
- `Home.md` - Wiki home page with navigation
- `Getting-Started.md` - Setup guide
- `Robot-Tuning-Guide.md` - Hardware tuning reference
- `General-Tuning-Guide.md` - Software tuning reference
- `Troubleshooting.md` - Common issues and solutions
- `_Sidebar.md` - Navigation sidebar (GitHub Wiki specific)

### GitBook Documentation
Located in: `/docs/` folder

Use GitBook for a polished, formatted online book:

1. Install GitBook: `npm install -g gitbook-cli`
2. Navigate to project root
3. Run: `gitbook serve`
4. Open http://localhost:4000 in your browser

Or deploy to GitBook.com for cloud hosting.

**Documentation Files:**
- `README.md` - GitBook documentation index
- `SUMMARY.md` - Table of contents (GitBook specific)
- `book.json` - GitBook configuration
- `introduction.md` - System overview and architecture
- `guides/robot-tuning-guide.md` - Comprehensive hardware tuning
- `guides/general-tuning-guide.md` - Comprehensive software tuning

## File Structure

```
FtcRobotController/
├── docs/                          # GitBook documentation
│   ├── README.md                 # Documentation index
│   ├── SUMMARY.md                # GitBook table of contents
│   ├── book.json                 # GitBook configuration
│   ├── introduction.md            # System introduction
│   └── guides/
│       ├── robot-tuning-guide.md    # Hardware tuning details
│       └── general-tuning-guide.md  # Software tuning details
│
└── wiki/                          # GitHub Wiki
    ├── Home.md                   # Wiki homepage
    ├── Getting-Started.md        # Setup instructions
    ├── Robot-Tuning-Guide.md     # Hardware reference
    ├── General-Tuning-Guide.md   # Software reference
    ├── Troubleshooting.md        # Troubleshooting guide
    └── _Sidebar.md               # Wiki navigation
```

## How to Use Each Format

### GitHub Wiki (Best for Quick Reference)
- Quick links and easy navigation
- Perfect for team members to check status
- Can be accessed directly without local setup
- Best for sharing quick tips and troubleshooting

**Access:** https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki

### GitBook (Best for Complete Documentation)
- Professional formatting and layout
- Better for in-depth learning
- Searchable and indexed
- Can be hosted statically or on GitBook.com
- Supports PDF export

**Local Access:** Run `gitbook serve` in project root

## Updating Documentation

When you need to update content:

1. **For Wiki**: Edit files in `/wiki/` folder
2. **For GitBook**: Edit files in `/docs/` folder
3. **Both versions**: Update both if the information should appear in both places

The guides in `/docs/guides/` are comprehensive and detailed.
The guides in `/wiki/` are quick references that link to the detailed guides.

## Configuration

### GitBook Configuration (book.json)
The `docs/book.json` file controls GitBook appearance and features:
- Title and description
- Plugins (GitHub links, anchors)
- Structure and organization

Edit this file to customize GitBook deployment settings.

### GitHub Wiki Configuration (_Sidebar.md)
The `wiki/_Sidebar.md` file provides navigation for GitHub Wiki.
Update navigation links as you add new wiki pages.

## Deploying Documentation

### Publishing to GitHub Wiki
1. Clone your repository locally
2. Push changes to `wiki/` folder
3. Changes appear automatically on GitHub wiki

### Publishing to GitBook.com

1. Sign up at [https://www.gitbook.com](https://www.gitbook.com)
2. Create a new space
3. Connect your GitHub repository
4. Select the `/docs` folder as your content source
5. GitBook will auto-publish on each commit

Alternatively, build static HTML:
```bash
gitbook build
# Output in _book/ folder
```

## Best Practices

1. **Keep content synchronized**: If information appears in both formats, keep it consistent
2. **Use anchors**: Help readers navigate between sections
3. **Add examples**: Use practical examples for configuration guides
4. **Update regularly**: Keep documentation current with code changes
5. **Gather feedback**: Ask team members what documentation would help them

## Quick Links

- **GitHub Repository**: [FtcRobotController on GitHub](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- **Official FTC Docs**: [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/)
- **FIRST Community**: [Community Forum](https://www.firstinspires.org/community/technical-support)

---

**Documentation Version**: 2025-2026 Season  
**Last Updated**: February 2026
