# Deployment Configuration Documentation

## Overview

This document describes the GitHub Actions workflow for automated deployment of the Physical AI & Humanoid Robotics textbook to GitHub Pages.

---

## Workflow Structure

### File Location

`.github/workflows/deploy.yml`

### Trigger Events

The workflow runs on:
1. **Push to main branch**: Automatic deployment when changes are merged/pushed to main
2. **Manual dispatch**: Can be triggered manually from GitHub Actions UI

```yaml
on:
  push:
    branches:
      - main
  workflow_dispatch:
```

### Permissions

Required permissions for GitHub Pages deployment:

- `contents: read` - Read repository contents
- `pages: write` - Write to GitHub Pages
- `id-token: write` - Generate deployment tokens

### Concurrency

```yaml
concurrency:
  group: "pages"
  cancel-in-progress: false
```

- **Group**: All deployments grouped under "pages"
- **Cancel in progress**: `false` - Waits for previous deployment to complete before starting new one

---

## Job 1: Build

### Purpose

Build the Docusaurus static site and validate output.

### Steps

#### 1. Checkout Repository

```yaml
- name: Checkout repository
  uses: actions/checkout@v4
  with:
    fetch-depth: 0  # Full history for last modified dates
```

**Why `fetch-depth: 0`**: Docusaurus uses Git history to display "Last updated" dates on pages.

#### 2. Setup Node.js

```yaml
- name: Setup Node.js
  uses: actions/setup-node@v4
  with:
    node-version: '18'
    cache: 'npm'
```

**Node version**: 18 LTS (matches local development)
**Cache**: npm dependencies cached for faster builds

#### 3. Install Dependencies

```yaml
- name: Install dependencies
  run: npm ci
```

**Why `npm ci`**: Clean install from package-lock.json (reproducible builds, faster than `npm install`)

#### 4. Build Site

```yaml
- name: Build site
  run: npm run build
  env:
    NODE_ENV: production
```

**Environment**: `NODE_ENV=production` enables optimizations (minification, tree-shaking)

**Build output**: `build/` directory with static files

#### 5. Validate Build Output

```yaml
- name: Validate build output
  run: |
    # Check that build directory exists
    if [ ! -d "build" ]; then
      echo "Error: build directory not found"
      exit 1
    fi

    # Check that search index was generated
    if [ ! -f "build/search-index.json" ]; then
      echo "Error: search index not generated"
      exit 1
    fi

    echo "Build validation passed"
```

**Validation checks**:
- Build directory exists
- Search index generated (proves search plugin worked)
- Broken links checked by Docusaurus (onBrokenLinks: 'throw')

**Exit codes**:
- `0` = Success
- `1` = Validation failure (stops deployment)

#### 6. Upload Build Artifact

```yaml
- name: Upload build artifact
  uses: actions/upload-pages-artifact@v3
  with:
    path: build
```

**Artifact**: Contains all static files (HTML, CSS, JS, images, search index)
**Retention**: 90 days (GitHub default)

---

## Job 2: Deploy

### Purpose

Deploy build artifact to GitHub Pages.

### Dependencies

```yaml
needs: build
```

**Requires**: Build job must complete successfully before deploy runs.

### Environment

```yaml
environment:
  name: github-pages
  url: ${{ steps.deployment.outputs.page_url }}
```

**Environment protection**: Can add approval requirements in GitHub Settings → Environments

**URL**: Automatically set to deployed site URL

### Steps

#### 1. Deploy to GitHub Pages

```yaml
- name: Deploy to GitHub Pages
  id: deployment
  uses: actions/deploy-pages@v4
```

**Action**: Official GitHub Pages deployment action
**Output**: `page_url` (deployed site URL)

#### 2. Success Notification

```yaml
- name: Deployment success notification
  if: success()
  run: |
    echo "✅ Deployment successful!"
    echo "Site URL: ${{ steps.deployment.outputs.page_url }}"
```

**Condition**: Only runs if deployment succeeds
**Output**: Logs deployment URL

#### 3. Failure Notification

```yaml
- name: Deployment failure notification
  if: failure()
  run: |
    echo "❌ Deployment failed!"
    echo "Check the logs above for errors"
    exit 1
```

**Condition**: Only runs if deployment fails
**Output**: Error message for troubleshooting

---

## Error Handling

### Build Failures

**Scenario 1: Missing images**

**Error**:
```
Error: Image not found: /img/missing-image.png
```

**Resolution**:
1. Check image path in Markdown file
2. Verify image exists in `static/img/` directory
3. Fix path or add missing image
4. Commit and push to main

---

**Scenario 2: Broken internal links**

**Error**:
```
Error: Docusaurus found broken links!
- Broken link on source page path = /docs/module-4-vla/intro-vla:
   -> linking to /docs/non-existent-page
```

**Resolution**:
1. Locate broken link in source file
2. Fix link path or create missing page
3. Commit and push to main

---

**Scenario 3: Invalid Mermaid diagrams**

**Error**:
```
Error: Could not parse Mermaid diagram
```

**Resolution**:
1. Test Mermaid syntax at https://mermaid.live
2. Fix diagram syntax
3. Commit and push to main

---

**Scenario 4: Search index not generated**

**Error**:
```
Error: search index not generated
```

**Resolution**:
1. Verify search plugin installed: `npm list @easyops-cn/docusaurus-search-local`
2. Check `docusaurus.config.ts` has `themes` section
3. Run local build: `npm run build` and check for errors
4. Fix configuration, commit, push

---

### Deployment Failures

**Scenario 1: GitHub Pages not enabled**

**Error**:
```
Error: GitHub Pages is not enabled for this repository
```

**Resolution**:
1. Go to repo Settings → Pages
2. Source: Select "GitHub Actions"
3. Save changes
4. Re-run workflow

---

**Scenario 2: Permissions error**

**Error**:
```
Error: Resource not accessible by integration
```

**Resolution**:
1. Check repo Settings → Actions → General
2. Workflow permissions: Select "Read and write permissions"
3. Enable "Allow GitHub Actions to create and approve pull requests"
4. Save and re-run workflow

---

**Scenario 3: Artifact upload timeout**

**Error**:
```
Error: Artifact upload timed out after 10 minutes
```

**Resolution**:
- Large build size (>500MB) may timeout
- Check for large files in `static/` directory
- Optimize images (compress with tools like TinyPNG)
- Remove unused assets

---

## Rollback Strategy

### Option 1: Revert Commit

**When**: Recent commit broke the site

**Steps**:
1. Identify bad commit: `git log --oneline`
2. Revert commit: `git revert <commit-hash>`
3. Push to main: `git push origin main`
4. Workflow runs automatically, deploys previous working version

---

### Option 2: Re-run Previous Workflow

**When**: Deployment failed but build was good

**Steps**:
1. Go to GitHub Actions tab
2. Find last successful workflow run
3. Click "Re-run all jobs"
4. Site redeploys with previous artifact

---

### Option 3: Manual Deployment from Branch

**When**: Need to deploy from non-main branch (emergency fix)

**Steps**:
1. Go to GitHub Actions → Deploy to GitHub Pages
2. Click "Run workflow"
3. Select branch from dropdown
4. Click "Run workflow"

---

## Configuration Files

### docusaurus.config.ts

**GitHub Pages Settings**:

```typescript
url: 'https://YOUR_GITHUB_USERNAME.github.io',
baseUrl: '/physical-ai-humanoid-robotics/',
organizationName: 'YOUR_GITHUB_USERNAME',
projectName: 'physical-ai-humanoid-robotics',
```

**TODO**: Replace `YOUR_GITHUB_USERNAME` with actual GitHub username before deploying.

**Base URL Rules**:
- For `https://username.github.io/repo-name/`: Use `/repo-name/`
- For `https://username.github.io/` (user site): Use `/`

---

## Testing Deployment

### Pre-Deployment Checklist

Before pushing to main:

- [ ] Run local build: `npm run build` (no errors)
- [ ] Test local site: `npm run serve` (all pages load)
- [ ] Test search: Enter queries, verify results
- [ ] Check images: All images render correctly
- [ ] Test navigation: All links work
- [ ] Test responsive design: Mobile, tablet, desktop views

---

### Post-Deployment Verification (T147-T152)

#### T148: Site Loads

**Test**: Visit `https://YOUR_GITHUB_USERNAME.github.io/physical-ai-humanoid-robotics/`

**Pass Criteria**:
- Homepage loads within 3 seconds
- All 12 chapters accessible via sidebar
- No 404 errors

---

#### T149: Internal Links Work

**Test**: Click "Previous Chapter" and "Next Chapter" links on 5 random pages

**Pass Criteria**:
- Links navigate to correct pages
- Anchor links jump to correct sections
- Module links work in footer

---

#### T150: Images Load

**Test**: Navigate to pages with images, check browser console

**Pass Criteria**:
- All images render (no broken image icons)
- No 404 errors in console
- Images load within 2 seconds

---

#### T151: Search Functionality

**Test**: Enter search queries: "ROS 2", "Gazebo", "Isaac Sim", "VLA"

**Pass Criteria**:
- Search bar visible in navbar
- Results appear within 1 second
- Keywords highlighted on result pages

---

#### T152: Responsive Design

**Test**: Use browser DevTools to test screen sizes

**Sizes**:
- Mobile: 320px width
- Tablet: 768px width
- Desktop: 1920px width

**Pass Criteria**:
- Sidebar collapses on mobile (hamburger menu)
- Text readable at all sizes
- Images scale appropriately
- No horizontal scrolling

---

## Monitoring Deployment

### GitHub Actions Dashboard

**Location**: Repository → Actions tab

**Metrics**:
- Build duration (target: <5 minutes)
- Deployment duration (target: <2 minutes)
- Success rate (target: >95%)
- Failed runs (investigate and fix)

### Deployment Logs

**Access**: Actions tab → Click workflow run → Click job

**Key sections**:
- Build site: Check for Docusaurus warnings
- Validate build: Verify checks passed
- Deploy to GitHub Pages: Deployment URL

### Error Notifications

**Email**: GitHub sends email on workflow failures (to commit author)

**Slack** (optional): Configure Slack webhook for notifications

```yaml
- name: Notify Slack on failure
  if: failure()
  uses: slackapi/slack-github-action@v1
  with:
    webhook-url: ${{ secrets.SLACK_WEBHOOK }}
    payload: |
      {
        "text": "❌ Deployment failed for ${{ github.repository }}"
      }
```

---

## Performance Optimization

### Build Time Optimization

**Current**: ~30 seconds for build

**Improvements**:
1. **Caching**: npm dependencies cached (already enabled)
2. **Incremental builds**: Docusaurus caches unchanged pages
3. **Parallel builds**: GitHub Actions runs on 2-core machines

---

### Deployment Time Optimization

**Current**: ~1-2 minutes for deployment

**Improvements**:
1. **Artifact compression**: Pages artifact automatically compressed
2. **CDN caching**: GitHub Pages uses CloudFlare CDN
3. **Concurrent uploads**: Pages action uploads chunks in parallel

---

## Security Considerations

### Secrets Management

**No secrets required** for this deployment (public textbook)

**If adding analytics**:
- Store API keys in GitHub Secrets (Settings → Secrets → Actions)
- Reference with `${{ secrets.SECRET_NAME }}`

### Dependency Security

**Dependabot**: Enabled by default, sends PRs for dependency updates

**Audit**: Run `npm audit` locally before pushing

---

## Maintenance

### Weekly Tasks

- [ ] Check GitHub Actions dashboard for failed runs
- [ ] Review Dependabot PRs, merge security updates
- [ ] Monitor deployment duration (alert if >10 minutes)

### Monthly Tasks

- [ ] Update Node.js version in workflow if LTS changes
- [ ] Review GitHub Actions usage (free tier: 2000 minutes/month)
- [ ] Check deployed site for broken links using tools like Broken Link Checker

### Quarterly Tasks

- [ ] Review and update workflow actions to latest versions
- [ ] Test rollback procedure (revert commit, verify deployment)
- [ ] Audit deployment logs for recurring warnings

---

## Troubleshooting Common Issues

### Issue 1: "This site can't be reached"

**Symptom**: Deployed URL returns DNS error

**Causes**:
1. GitHub Pages not enabled
2. Incorrect baseUrl in config
3. DNS propagation delay (wait 10 minutes)

**Resolution**: Check Settings → Pages, verify Source is "GitHub Actions"

---

### Issue 2: Assets not loading (404)

**Symptom**: Images/CSS/JS return 404

**Cause**: Incorrect baseUrl in `docusaurus.config.ts`

**Resolution**:
- For `/repo-name/`: Set `baseUrl: '/repo-name/'`
- Rebuild and redeploy

---

### Issue 3: Search not working on deployed site

**Symptom**: Search bar visible but no results

**Cause**: Search index not included in deployment

**Resolution**:
1. Check build logs for search index generation
2. Verify `build/search-index.json` exists after build
3. Rebuild and redeploy

---

## References

- **GitHub Pages Docs**: https://docs.github.com/en/pages
- **GitHub Actions Docs**: https://docs.github.com/en/actions
- **Docusaurus Deployment**: https://docusaurus.io/docs/deployment#deploying-to-github-pages
- **Deploy Pages Action**: https://github.com/actions/deploy-pages

---

**Last Updated**: 2025-12-02
**Workflow Version**: 1.0
**Tested With**: GitHub Actions, Node.js 18, Docusaurus 3.9.2
