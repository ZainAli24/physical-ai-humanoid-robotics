# Search Configuration Documentation

## Plugin Selection Rationale

### Chosen Solution: @easyops-cn/docusaurus-search-local

**Plugin**: `@easyops-cn/docusaurus-search-local` v0.44.0+

**Rationale**:
1. **Local Search**: No external dependencies or API keys required (vs. Algolia DocSearch)
2. **Zero Cost**: Free for all projects (Algolia DocSearch requires approval + rate limits)
3. **Privacy**: Search index generated locally, no data sent to third parties
4. **Performance**: Sub-100ms search response time for local index
5. **Offline Support**: Works without internet connection once site is built
6. **Ease of Setup**: Simple npm install + configuration, no external service setup
7. **Full Control**: Complete control over search indexing and customization

### Alternatives Considered

**Algolia DocSearch**:
- ❌ Requires application and approval process
- ❌ Rate limits (10k searches/month free tier)
- ❌ External dependency (Algolia service must be available)
- ✅ More advanced features (typo tolerance, synonyms)
- **Decision**: Rejected due to setup complexity and external dependencies

**Lunr.js (Raw Integration)**:
- ✅ Lightweight and fast
- ❌ Requires manual integration and indexing
- ❌ No out-of-the-box Docusaurus support
- **Decision**: Rejected in favor of ready-made plugin

---

## Configuration Options

### Plugin Configuration (docusaurus.config.ts)

```typescript
themes: [
  [
    require.resolve('@easyops-cn/docusaurus-search-local'),
    {
      hashed: true,                              // Enable hashed search files for caching
      language: ['en'],                          // Search language (English only)
      indexDocs: true,                           // Index all documentation pages
      indexBlog: false,                          // Blog disabled in our setup
      indexPages: false,                         // Only index docs, not static pages
      docsRouteBasePath: '/docs',                // Base path for documentation
      highlightSearchTermsOnTargetPage: true,    // Highlight search terms on result pages
      searchResultLimits: 8,                     // Max 8 results per query
      searchResultContextMaxLength: 50,          // 50 chars of context per result
    },
  ],
],
```

### Configuration Parameters Explained

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `hashed` | `true` | Adds hash to search index files for cache busting when content changes |
| `language` | `['en']` | English-only search (matches i18n config) |
| `indexDocs` | `true` | Index all Markdown/MDX files in `/docs` directory |
| `indexBlog` | `false` | Blog feature disabled (textbook has no blog) |
| `indexPages` | `false` | Only docs are searchable (excludes homepage, custom pages) |
| `docsRouteBasePath` | `'/docs'` | URL base path for documentation routes |
| `highlightSearchTermsOnTargetPage` | `true` | Yellow highlight on search terms when user clicks result |
| `searchResultLimits` | `8` | Show maximum 8 results per query (prevents UI clutter) |
| `searchResultContextMaxLength` | `50` | Show 50 characters of surrounding context for each result |

---

## Search Index Coverage

### Indexed Content

The search plugin will index the following:

**Module 1: ROS 2 Fundamentals** (3 chapters):
- `docs/module-1-ros2/ros2-fundamentals.md`
- `docs/module-1-ros2/ros2-nodes-topics.md`
- `docs/module-1-ros2/ros2-services-actions.md`

**Module 2: Gazebo & Unity Simulation** (3 chapters):
- `docs/module-2-gazebo-unity/intro-gazebo.md`
- `docs/module-2-gazebo-unity/unity-robotics.md`
- `docs/module-2-gazebo-unity/simulation-best-practices.md`

**Module 3: NVIDIA Isaac** (3 chapters):
- `docs/module-3-isaac/intro-isaac.md`
- `docs/module-3-isaac/isaac-sim.md`
- `docs/module-3-isaac/isaac-gym-rl.md`

**Module 4: Vision-Language-Action Models** (3 chapters):
- `docs/module-4-vla/intro-vla.md`
- `docs/module-4-vla/vla-architectures.md`
- `docs/module-4-vla/vla-training-deployment.md`

**Additional Pages**:
- `docs/preface.md`
- `docs/intro-physical-ai.md`

**Total**: 14 pages across 4 modules + 2 introductory pages = **16 indexed pages**

### Searchable Content Types

- **Headings**: All H1-H6 headings indexed with higher weight
- **Body Text**: Full paragraph text indexed
- **Code Comments**: Inline comments in code blocks
- **List Items**: Bullet points and numbered lists
- **Table Content**: Text within Markdown tables
- **Link Text**: Anchor text for internal/external links

### Not Indexed

- **Code Blocks**: Raw code syntax (too noisy for search)
- **Mermaid Diagrams**: SVG/graph definitions (non-textual)
- **Images**: Alt text indexed, but not image content
- **Frontmatter**: YAML metadata (title/description used, but not searchable)

---

## Testing Steps

### 1. Build Search Index

```bash
npm run build
```

**Expected Output**:
```
[SUCCESS] Generated static files in "build".
[INFO] Search index generated at: build/search-index.json
```

**Validation**:
- Check `build/search-index.json` exists (should be ~500KB-2MB for 16 pages)
- Verify no build errors related to search plugin

---

### 2. Test Search Locally

```bash
npm start
```

**Expected Behavior**:
- Search bar appears in top-right navbar on all pages
- Search icon (🔍) visible and clickable

---

### 3. Search Query Tests

#### Test Case 1: Basic Keyword Search

**Query**: `ROS 2`

**Expected Results**:
- 5-8 results from Module 1 chapters
- Results show: "ROS 2 Fundamentals", "ROS 2 Nodes and Topics", etc.
- Keyword "ROS 2" highlighted in yellow on result pages

**Pass Criteria**: Results appear within 1 second, at least 3 relevant results

---

#### Test Case 2: Tool-Specific Search

**Query**: `Gazebo`

**Expected Results**:
- Results from Module 2 chapters
- Mentions of Gazebo launch files, world files, plugins
- Context snippets show surrounding text

**Pass Criteria**: At least 2 results from Module 2

---

#### Test Case 3: Advanced Topic Search

**Query**: `Isaac Gym`

**Expected Results**:
- Results from Module 3 chapters
- Mentions of reinforcement learning, GPU acceleration, parallel environments
- Chapter titles visible in search results

**Pass Criteria**: At least 1 result from Module 3 Chapter 3

---

#### Test Case 4: Architecture Search

**Query**: `VLA`

**Expected Results**:
- Results from Module 4 chapters
- Mentions of RT-1, RT-2, PaLM-E, vision-language-action models
- Architecture diagrams context

**Pass Criteria**: At least 2 results from Module 4

---

#### Test Case 5: Exact Phrase Search

**Query**: `"ros2 topic pub"`

**Expected Results**:
- Exact match results only (quoted phrase search)
- Results from ROS 2 chapters mentioning this command
- Code block context shown

**Pass Criteria**: At least 1 result with exact phrase

---

#### Test Case 6: No Results Handling

**Query**: `kubernetes docker tensorflow`

**Expected Results**:
- "No results found" message displayed
- Suggestion to refine search query
- No errors or blank screen

**Pass Criteria**: Graceful "no results" message

---

#### Test Case 7: Special Characters

**Query**: `C++ python ros2`

**Expected Results**:
- Search handles special characters (++) correctly
- Results for programming language mentions
- No parsing errors

**Pass Criteria**: Search executes without errors

---

#### Test Case 8: Case Insensitivity

**Query**: `isaac sim` (lowercase) vs. `Isaac Sim` (capitalized)

**Expected Results**:
- Both queries return identical results
- Case-insensitive search behavior

**Pass Criteria**: Same results for both queries

---

### 4. Performance Testing

**Metric**: Search Response Time

**Test**: Enter query and measure time until results appear

**Pass Criteria**:
- Queries under 20 characters: < 100ms response
- Queries 20-50 characters: < 500ms response
- Queries over 50 characters: < 1000ms response

**Tool**: Browser DevTools Network tab (measure time from keypress to result render)

---

### 5. UI/UX Testing

#### Search Bar Visibility

**Test**: Navigate to 5 random pages

**Pass Criteria**:
- Search bar visible in navbar on all pages
- Search icon (🔍) clearly visible
- Search input placeholder text: "Search docs"

#### Result Navigation

**Test**: Click on search result

**Pass Criteria**:
- Page loads to correct section
- Search terms highlighted in yellow on target page
- Highlight fades after 3 seconds

#### Keyboard Navigation

**Test**: Use Tab/Enter keys to navigate search

**Pass Criteria**:
- Tab: Move between search results
- Enter: Open selected result
- Escape: Close search modal

---

## Troubleshooting

### Issue 1: Search Index Not Generated

**Symptom**: `build/search-index.json` missing after build

**Solution**:
1. Verify plugin installed: `npm list @easyops-cn/docusaurus-search-local`
2. Check `docusaurus.config.ts` has `themes` section
3. Run `npm run clear` then `npm run build`

---

### Issue 2: Search Bar Not Visible

**Symptom**: No search icon in navbar

**Solution**:
1. Check browser console for JavaScript errors
2. Verify plugin theme is loaded (not overridden by custom theme)
3. Clear browser cache and hard refresh (Ctrl+Shift+R)

---

### Issue 3: No Results for Known Content

**Symptom**: Query returns 0 results for content that exists

**Solution**:
1. Check `indexDocs: true` in config
2. Verify `docsRouteBasePath: '/docs'` matches actual docs path
3. Rebuild search index: `npm run build`

---

### Issue 4: Search Too Slow

**Symptom**: Response time > 1 second

**Solution**:
1. Reduce `searchResultLimits` from 8 to 5
2. Decrease `searchResultContextMaxLength` from 50 to 30
3. Enable `hashed: true` for caching (already enabled)

---

## Maintenance

### Updating Search Index

**When**: After adding/modifying any documentation

**How**: Run `npm run build` to regenerate search index

**Frequency**: Automatic on every build (no manual maintenance)

### Monitoring Search Usage

**Metrics** (if analytics added):
- Top 10 search queries
- Queries with 0 results (identify missing content)
- Average query length
- Click-through rate on search results

**Tools**: Google Analytics (if integrated) or Plausible Analytics

---

## Future Enhancements

### Potential Improvements

1. **Synonym Support**: Map "RL" → "Reinforcement Learning", "VLA" → "Vision Language Action"
2. **Search Analytics**: Track popular queries to improve content
3. **Autocomplete**: Suggest queries as user types
4. **Filters**: Filter by module (Module 1, Module 2, etc.)
5. **Search History**: Show recent searches in dropdown

### Plugin Updates

**Current Version**: 0.44.0+ (as of December 2024)

**Update Policy**:
- Check for updates monthly: `npm outdated @easyops-cn/docusaurus-search-local`
- Test search functionality after updates
- Review changelog for breaking changes

---

## References

- **Plugin Docs**: https://github.com/easyops-cn/docusaurus-search-local
- **Docusaurus Themes**: https://docusaurus.io/docs/using-themes
- **Search Best Practices**: https://docusaurus.io/docs/search

---

**Last Updated**: 2025-12-02
**Configuration Version**: 1.0
**Tested With**: Docusaurus 3.9.2, Node.js 22.18.0
